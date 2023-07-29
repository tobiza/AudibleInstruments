#include "plugin.hpp"

#pragma GCC diagnostic push
#ifndef __clang__
	#pragma GCC diagnostic ignored "-Wsuggest-override"
#endif
#include "plaits/dsp/voice.h"
//#include "plaits/user_data_receiver.h"
#include "plaits/user_data.h"
#include "stmlib/dsp/hysteresis_quantizer.h"
#pragma GCC diagnostic pop

#include <osdialog.h>
#include <thread>

#include <fstream>
#include <iterator>

static const char WAVE_FILTERS[] = "BIN (*.bin):bin, BIN";
static std::string waveDir;

struct Plaits : Module {
	enum ParamIds {
		MODEL1_PARAM,
		MODEL2_PARAM,
		FREQ_PARAM,
		HARMONICS_PARAM,
		TIMBRE_PARAM,
		MORPH_PARAM,
		TIMBRE_CV_PARAM,
		FREQ_CV_PARAM,
		MORPH_CV_PARAM,
		LPG_COLOR_PARAM,
		LPG_DECAY_PARAM,
		FREQ_ROOT_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		ENGINE_INPUT,
		TIMBRE_INPUT,
		FREQ_INPUT,
		MORPH_INPUT,
		HARMONICS_INPUT,
		TRIGGER_INPUT,
		LEVEL_INPUT,
		NOTE_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		OUT_OUTPUT,
		AUX_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		ENUMS(MODEL_LIGHT, 8 * 2),
		NUM_LIGHTS
	};

	plaits::Voice voice[16];
	plaits::Patch patch = {};
	plaits::UserData user_data;
	char shared_buffer[16][16384] = {};
	float triPhase = 0.f;
	int frequencyMode = 10;
	stmlib::HysteresisQuantizer2 octaveQuantizer;

	dsp::SampleRateConverter<16 * 2> outputSrc;
	dsp::DoubleRingBuffer<dsp::Frame<16 * 2>, 256> outputBuffer;
	bool lowCpu = false;

	dsp::BooleanTrigger model1Trigger;
	dsp::BooleanTrigger model2Trigger;

	bool loading = false;

	Plaits() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configButton(MODEL1_PARAM, "Previous model");
		configButton(MODEL2_PARAM, "Next model");
		configParam(FREQ_PARAM, -4.0, 4.0, 0.0, "Frequency", " semitones", 0.f, 12.f);
		configParam(FREQ_ROOT_PARAM, -4.0, 4.0, 0.0, "Frequency Root", " semitones", 0.f, 12.f);
		configParam(HARMONICS_PARAM, 0.0, 1.0, 0.5, "Harmonics", "%", 0.f, 100.f);
		configParam(TIMBRE_PARAM, 0.0, 1.0, 0.5, "Timbre", "%", 0.f, 100.f);
		configParam(LPG_COLOR_PARAM, 0.0, 1.0, 0.5, "Lowpass gate response", "%", 0.f, 100.f);
		configParam(MORPH_PARAM, 0.0, 1.0, 0.5, "Morph", "%", 0.f, 100.f);
		configParam(LPG_DECAY_PARAM, 0.0, 1.0, 0.5, "Lowpass gate decay", "%", 0.f, 100.f);
		configParam(TIMBRE_CV_PARAM, -1.0, 1.0, 0.0, "Timbre CV");
		configParam(FREQ_CV_PARAM, -1.0, 1.0, 0.0, "Frequency CV");
		configParam(MORPH_CV_PARAM, -1.0, 1.0, 0.0, "Morph CV");

		configInput(ENGINE_INPUT, "Model");
		configInput(TIMBRE_INPUT, "Timbre");
		configInput(FREQ_INPUT, "FM");
		configInput(MORPH_INPUT, "Morph");
		configInput(HARMONICS_INPUT, "Harmonics");
		configInput(TRIGGER_INPUT, "Trigger");
		configInput(LEVEL_INPUT, "Level");
		configInput(NOTE_INPUT, "Pitch (1V/oct)");

		configOutput(OUT_OUTPUT, "Main");
		configOutput(AUX_OUTPUT, "Auxiliary");

		for (int i = 0; i < 16; i++) {
			stmlib::BufferAllocator allocator(shared_buffer[i], sizeof(shared_buffer[i]));
			voice[i].Init(&allocator, &user_data);
		}

		octaveQuantizer.Init(9, 0.01f, false);

		onReset();
	}

	void onReset() override {
		patch.engine = 0;
		patch.lpg_colour = 0.5f;
		patch.decay = 0.5f;
	}

	void onRandomize() override {
		patch.engine = random::u32() % 16;
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();

		json_object_set_new(rootJ, "lowCpu", json_boolean(lowCpu));
		json_object_set_new(rootJ, "model", json_integer(patch.engine));

		const uint8_t* userDataBuffer = user_data.getBuffer();
		if (userDataBuffer != nullptr) {
			std::string userDataString = rack::string::toBase64(userDataBuffer, plaits::UserData::MAX_USER_DATA_SIZE);
			json_object_set_new(rootJ, "userData", json_string(userDataString.c_str()));
		}

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* lowCpuJ = json_object_get(rootJ, "lowCpu");
		if (lowCpuJ)
			lowCpu = json_boolean_value(lowCpuJ);

		json_t* modelJ = json_object_get(rootJ, "model");
		if (modelJ)
			patch.engine = json_integer_value(modelJ);

		json_t* userDataJ = json_object_get(rootJ, "userData");
		if (userDataJ) {
			std::string userDataString = json_string_value(userDataJ);
			const std::vector<uint8_t> userDataVector = rack::string::fromBase64(userDataString);
			if (userDataVector.size() > 0) {
				const uint8_t* userDataBuffer = &userDataVector[0];
				user_data.setBuffer(userDataBuffer);
			}
		}

		// Legacy <=1.0.2
		json_t* lpgColorJ = json_object_get(rootJ, "lpgColor");
		if (lpgColorJ)
			params[LPG_COLOR_PARAM].setValue(json_number_value(lpgColorJ));

		// Legacy <=1.0.2
		json_t* decayJ = json_object_get(rootJ, "decay");
		if (decayJ)
			params[LPG_DECAY_PARAM].setValue(json_number_value(decayJ));
	}

	void process(const ProcessArgs& args) override {
		int channels = std::max(inputs[NOTE_INPUT].getChannels(), 1);

		if (outputBuffer.empty()) {
			const int blockSize = 12;

			// Model buttons
			if (model1Trigger.process(params[MODEL1_PARAM].getValue())) {
				if (patch.engine == 0) {
					patch.engine = 24 - 1;
				} else {
					patch.engine = (patch.engine - 1) % 24;
				}
			}
			if (model2Trigger.process(params[MODEL2_PARAM].getValue())) {
				patch.engine = (patch.engine + 1) % 24;
			}

			// Model lights
			// Pulse light at 2 Hz
			triPhase += 2.f * args.sampleTime * blockSize;
			if (triPhase >= 1.f)
				triPhase -= 1.f;
			float tri = (triPhase < 0.5f) ? triPhase * 2.f : (1.f - triPhase) * 2.f;

			// Get active engines of all voice channels
			bool activeLights[16] = {};
			bool pulse = false;
			for (int c = 0; c < channels; c++) {
				int activeEngine = voice[c].active_engine();
				if (activeEngine < 8) {
					activeLights[activeEngine] = true;
					activeLights[activeEngine+8] = true;
				} else {
					activeLights[activeEngine-8] = true;
				}
				// Pulse the light if at least one voice is using a different engine.
				if (activeEngine != patch.engine)
					pulse = true;
			}

			// Set model lights
			for (int i = 0; i < 16; i++) {
				// Transpose the [light][color] table
				int lightId = (i % 8) * 2 + (i / 8);
				float brightness = activeLights[i];
				if (patch.engine == (i+8) && pulse)		// TODO: fix with orange colors
					brightness = tri;
				lights[MODEL_LIGHT + lightId].setBrightness(brightness);
			}

			// Calculate pitch for lowCpu mode if needed
			float pitch = params[FREQ_PARAM].getValue();
			if (lowCpu)
				pitch += std::log2(48000.f * args.sampleTime);
			// Update patch

			// Similar implementation to original Plaits ui.cc code.
			// TODO: check with lowCpu mode.
			if (frequencyMode == 0) {
				patch.note = -48.37f + pitch * 15.f;
			} else if (frequencyMode == 9) {
				float fineTune = params[FREQ_ROOT_PARAM].getValue() / 4.f;
				patch.note = 53.f + fineTune * 14.f + 12.f * static_cast<float>(octaveQuantizer.Process(0.5f * pitch / 4.f + 0.5f) - 4.f);
			} else if (frequencyMode == 10) {
				patch.note = 60.f + pitch * 12.f;
			} else {
				patch.note = static_cast<float>(frequencyMode) * 12.f + pitch * 7.f / 4.f;
			}

			patch.harmonics = params[HARMONICS_PARAM].getValue();
			patch.timbre = params[TIMBRE_PARAM].getValue();
			patch.morph = params[MORPH_PARAM].getValue();
			patch.lpg_colour = params[LPG_COLOR_PARAM].getValue();
			patch.decay = params[LPG_DECAY_PARAM].getValue();
			patch.frequency_modulation_amount = params[FREQ_CV_PARAM].getValue();
			patch.timbre_modulation_amount = params[TIMBRE_CV_PARAM].getValue();
			patch.morph_modulation_amount = params[MORPH_CV_PARAM].getValue();

			// Render output buffer for each voice
			dsp::Frame<16 * 2> outputFrames[blockSize];
			for (int c = 0; c < channels; c++) {
				// Construct modulations
				plaits::Modulations modulations;
				modulations.engine = inputs[ENGINE_INPUT].getPolyVoltage(c) / 5.f;
				modulations.note = inputs[NOTE_INPUT].getVoltage(c) * 12.f;
				modulations.frequency = inputs[FREQ_INPUT].getPolyVoltage(c) * 6.f;
				modulations.harmonics = inputs[HARMONICS_INPUT].getPolyVoltage(c) / 5.f;
				modulations.timbre = inputs[TIMBRE_INPUT].getPolyVoltage(c) / 8.f;
				modulations.morph = inputs[MORPH_INPUT].getPolyVoltage(c) / 8.f;
				// Triggers at around 0.7 V
				modulations.trigger = inputs[TRIGGER_INPUT].getPolyVoltage(c) / 3.f;
				modulations.level = inputs[LEVEL_INPUT].getPolyVoltage(c) / 8.f;

				modulations.frequency_patched = inputs[FREQ_INPUT].isConnected();
				modulations.timbre_patched = inputs[TIMBRE_INPUT].isConnected();
				modulations.morph_patched = inputs[MORPH_INPUT].isConnected();
				modulations.trigger_patched = inputs[TRIGGER_INPUT].isConnected();
				modulations.level_patched = inputs[LEVEL_INPUT].isConnected();

				// Render frames
				plaits::Voice::Frame output[blockSize];
				voice[c].Render(patch, modulations, output, blockSize);

				// Convert output to frames
				for (int i = 0; i < blockSize; i++) {
					outputFrames[i].samples[c * 2 + 0] = output[i].out / 32768.f;
					outputFrames[i].samples[c * 2 + 1] = output[i].aux / 32768.f;
				}
			}

			// Convert output
			if (lowCpu) {
				int len = std::min((int) outputBuffer.capacity(), blockSize);
				std::memcpy(outputBuffer.endData(), outputFrames, len * sizeof(outputFrames[0]));
				outputBuffer.endIncr(len);
			}
			else {
				outputSrc.setRates(48000, (int) args.sampleRate);
				int inLen = blockSize;
				int outLen = outputBuffer.capacity();
				outputSrc.setChannels(channels * 2);
				outputSrc.process(outputFrames, &inLen, outputBuffer.endData(), &outLen);
				outputBuffer.endIncr(outLen);
			}
		}

		// Set output
		if (!outputBuffer.empty()) {
			dsp::Frame<16 * 2> outputFrame = outputBuffer.shift();
			for (int c = 0; c < channels; c++) {
				// Inverting op-amp on outputs
				outputs[OUT_OUTPUT].setVoltage(-outputFrame.samples[c * 2 + 0] * 5.f, c);
				outputs[AUX_OUTPUT].setVoltage(-outputFrame.samples[c * 2 + 1] * 5.f, c);
			}
		}
		outputs[OUT_OUTPUT].setChannels(channels);
		outputs[AUX_OUTPUT].setChannels(channels);
	}

	void reset() {
			bool success = user_data.Save(nullptr, patch.engine);
			if (success) {
				for (int c = 0; c < 16; c++) {
					voice[c].ReloadUserData();
				}
			}
	}

	void load(const std::string& path) {
		loading = true;
		DEFER({loading = false;});
		// HACK Sleep 100us so DSP thread is likely to finish processing before we resize the vector
		std::this_thread::sleep_for(std::chrono::duration<double>(100e-6));

		std::string ext = string::lowercase(system::getExtension(path));
		
		if (ext == ".bin") {
			std::ifstream input(path, std::ios::binary);
			std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(input), {});
			uint8_t* rx_buffer = buffer.data();
			bool success = user_data.Save(rx_buffer, patch.engine);
			if (success) {
				for (int c = 0; c < 16; c++) {
					voice[c].ReloadUserData();
				}
			}
		}
	}

	void loadDialog() {
		osdialog_filters* filters = osdialog_filters_parse(WAVE_FILTERS);
		char* pathC = osdialog_file(OSDIALOG_OPEN, waveDir.empty() ? NULL : waveDir.c_str(), NULL, filters);
		if (!pathC) {
			// Fail silently
			return;
		}
		const std::string path = pathC;
		std::free(pathC);

 		waveDir = system::getDirectory(path);
		load(path);
	}
};

static const std::string modelLabels[24] = {
	"Classic waveshapes with filter",
	"Phase distortion",
	"6-operator FM 1",
	"6-operator FM 2",
	"6-operator FM 3",
	"Wave terrain synthesis",
	"String machine",
	"Chiptune",
	"Pair of classic waveforms",
	"Waveshaping oscillator",
	"Two operator FM",
	"Granular formant oscillator",
	"Harmonic oscillator",
	"Wavetable oscillator",
	"Chords",
	"Vowel and speech synthesis",
	"Granular cloud",
	"Filtered noise",
	"Particle noise",
	"Inharmonic string modeling",
	"Modal resonator",
	"Analog bass drum",
	"Analog snare drum",
	"Analog hi-hat",
};

static const std::string frequencyModes[11] = {
	"LFO mode",
	"C0 +/- 7 semitones",
	"C1 +/- 7 semitones",
	"C2 +/- 7 semitones",
	"C3 +/- 7 semitones",
	"C4 +/- 7 semitones",
	"C5 +/- 7 semitones",
	"C6 +/- 7 semitones",
	"C7 +/- 7 semitones",
	"Octaves",
	"C0 to C8",
};

struct PlaitsWidget : ModuleWidget {
	bool lpgMode = false;
	bool freqRootMode = false;

	PlaitsWidget(Plaits* module) {
		setModule(module);
		setPanel(Svg::load(asset::plugin(pluginInstance, "res/Plaits.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<TL1105>(mm2px(Vec(23.32685, 14.6539)), module, Plaits::MODEL1_PARAM));
		addParam(createParam<TL1105>(mm2px(Vec(32.22764, 14.6539)), module, Plaits::MODEL2_PARAM));
		addParam(createParam<Rogan3PSWhite>(mm2px(Vec(3.1577, 20.21088)), module, Plaits::FREQ_PARAM));
		addParam(createParam<Rogan3PSWhite>(mm2px(Vec(39.3327, 20.21088)), module, Plaits::HARMONICS_PARAM));
		addParam(createParam<Rogan1PSWhite>(mm2px(Vec(4.04171, 49.6562)), module, Plaits::TIMBRE_PARAM));
		addParam(createParam<Rogan1PSWhite>(mm2px(Vec(42.71716, 49.6562)), module, Plaits::MORPH_PARAM));
		addParam(createParam<Trimpot>(mm2px(Vec(7.88712, 77.60705)), module, Plaits::TIMBRE_CV_PARAM));
		addParam(createParam<Trimpot>(mm2px(Vec(27.2245, 77.60705)), module, Plaits::FREQ_CV_PARAM));
		addParam(createParam<Trimpot>(mm2px(Vec(46.56189, 77.60705)), module, Plaits::MORPH_CV_PARAM));

		ParamWidget* lpgColorParam = createParam<Rogan1PSBlue>(mm2px(Vec(4.04171, 49.6562)), module, Plaits::LPG_COLOR_PARAM);
		lpgColorParam->hide();
		addParam(lpgColorParam);
		ParamWidget* decayParam = createParam<Rogan1PSBlue>(mm2px(Vec(42.71716, 49.6562)), module, Plaits::LPG_DECAY_PARAM);
		decayParam->hide();
		addParam(decayParam);
		ParamWidget* freqRootParam = createParam<Rogan3PSRed>(mm2px(Vec(3.1577, 20.21088)), module, Plaits::FREQ_ROOT_PARAM);
		freqRootParam->hide();
		addParam(freqRootParam);

		addInput(createInput<PJ301MPort>(mm2px(Vec(3.31381, 92.48067)), module, Plaits::ENGINE_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(14.75983, 92.48067)), module, Plaits::TIMBRE_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(26.20655, 92.48067)), module, Plaits::FREQ_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(37.65257, 92.48067)), module, Plaits::MORPH_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(49.0986, 92.48067)), module, Plaits::HARMONICS_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(3.31381, 107.08103)), module, Plaits::TRIGGER_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(14.75983, 107.08103)), module, Plaits::LEVEL_INPUT));
		addInput(createInput<PJ301MPort>(mm2px(Vec(26.20655, 107.08103)), module, Plaits::NOTE_INPUT));

		addOutput(createOutput<PJ301MPort>(mm2px(Vec(37.65257, 107.08103)), module, Plaits::OUT_OUTPUT));
		addOutput(createOutput<PJ301MPort>(mm2px(Vec(49.0986, 107.08103)), module, Plaits::AUX_OUTPUT));

		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 23.31649)), module, Plaits::MODEL_LIGHT + 0 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 28.71704)), module, Plaits::MODEL_LIGHT + 1 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 34.1162)), module, Plaits::MODEL_LIGHT + 2 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 39.51675)), module, Plaits::MODEL_LIGHT + 3 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 44.91731)), module, Plaits::MODEL_LIGHT + 4 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 50.31785)), module, Plaits::MODEL_LIGHT + 5 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 55.71771)), module, Plaits::MODEL_LIGHT + 6 * 2));
		addChild(createLight<MediumLight<GreenRedLight>>(mm2px(Vec(28.79498, 61.11827)), module, Plaits::MODEL_LIGHT + 7 * 2));
	}

	void appendContextMenu(Menu* menu) override {
		Plaits* module = dynamic_cast<Plaits*>(this->module);

		menu->addChild(new MenuSeparator);

		menu->addChild(createBoolPtrMenuItem("Low CPU (disable resampling)", "", &module->lowCpu));

		menu->addChild(createBoolMenuItem("Edit LPG response/decay", "",
			[=]() {return this->getLpgMode();},
			[=](bool val) {this->setLpgMode(val);}
		));

		menu->addChild(new MenuSeparator);

		menu->addChild(createSubmenuItem("Frequency mode", "", [=](Menu* menu) {
			for (int i = 0; i < 11; i++) {
			menu->addChild(createCheckMenuItem(frequencyModes[i], "",
				[=]() {return module->frequencyMode == i;},
				[=]() {module->frequencyMode = i;}
			));
		}
		}));

		menu->addChild(createBoolMenuItem("Edit frequency root", "",
			[=]() {return this->getFreqRootMode();},
			[=](bool val) {this->setFreqRootMode(val);}
		));

		menu->addChild(new MenuSeparator);

		menu->addChild(createMenuItem("Reset custom data for current engine", "",
			[=]() {module->reset();}
		));

		menu->addChild(createMenuItem("Load custom data for current engine", "",
			[=]() {module->loadDialog();}
		));

		menu->addChild(new MenuSeparator);

		menu->addChild(createSubmenuItem("Pitched models", "", [=](Menu* menu) {
			for (int i = 8; i < 16; i++) {
			menu->addChild(createCheckMenuItem(modelLabels[i], "",
				[=]() {return module->patch.engine == i;},
				[=]() {module->patch.engine = i;}
			));
		}
		}));

		menu->addChild(createSubmenuItem("Noise/percussive models", "", [=](Menu* menu) {
		for (int i = 16; i < 24; i++) {
			menu->addChild(createCheckMenuItem(modelLabels[i], "",
				[=]() {return module->patch.engine == i;},
				[=]() {module->patch.engine = i;}
			));
		}
		}));

		menu->addChild(createSubmenuItem("New synthesis models", "", [=](Menu* menu) {
		for (int i = 0; i < 8; i++) {
			menu->addChild(createCheckMenuItem(modelLabels[i], "",
				[=]() {return module->patch.engine == i;},
				[=]() {module->patch.engine = i;}
			));
		}
		}));
	}

	void setLpgMode(bool lpgMode) {
		// ModuleWidget::getParam() doesn't work if the ModuleWidget doesn't have a module.
		if (!module)
			return;
		if (lpgMode) {
			getParam(Plaits::MORPH_PARAM)->hide();
			getParam(Plaits::TIMBRE_PARAM)->hide();
			getParam(Plaits::LPG_DECAY_PARAM)->show();
			getParam(Plaits::LPG_COLOR_PARAM)->show();
		}
		else {
			getParam(Plaits::MORPH_PARAM)->show();
			getParam(Plaits::TIMBRE_PARAM)->show();
			getParam(Plaits::LPG_DECAY_PARAM)->hide();
			getParam(Plaits::LPG_COLOR_PARAM)->hide();
		}
		this->lpgMode = lpgMode;
	}

	bool getLpgMode() {
		return this->lpgMode;
	}

	void setFreqRootMode(bool freqRootMode) {
		// ModuleWidget::getParam() doesn't work if the ModuleWidget doesn't have a module.
		if (!module)
			return;
		if (freqRootMode) {
			getParam(Plaits::FREQ_PARAM)->hide();
			getParam(Plaits::FREQ_ROOT_PARAM)->show();
		}
		else {
			getParam(Plaits::FREQ_PARAM)->show();
			getParam(Plaits::FREQ_ROOT_PARAM)->hide();
		}
		this->freqRootMode = freqRootMode;
	}

	bool getFreqRootMode() {
		return this->freqRootMode;
	}
};


Model* modelPlaits = createModel<Plaits, PlaitsWidget>("Plaits");

PACKAGES       = plaits/test stmlib/utils plaits plaits/dsp plaits/dsp/chords plaits/dsp/engine plaits/dsp/engine2 plaits/dsp/fm stmlib/dsp plaits/dsp/speech plaits/dsp/physical_modelling stm_audio_bootloader/fsk

VPATH          = $(PACKAGES)

TARGET         = plaits_test
BUILD_ROOT     = build/
BUILD_DIR      = $(BUILD_ROOT)$(TARGET)/
CC_FILES       = algorithms.cc \
		additive_engine.cc \
		bass_drum_engine.cc \
		chiptune_engine.cc \
		chord_bank.cc \
		chord_engine.cc \
		dx_units.cc \
		fm_engine.cc \
		grain_engine.cc \
		hi_hat_engine.cc \
		lpc_speech_synth.cc \
		lpc_speech_synth_controller.cc \
		lpc_speech_synth_phonemes.cc \
		lpc_speech_synth_words.cc \
		modal_engine.cc \
		modal_voice.cc \
		naive_speech_synth.cc \
		noise_engine.cc \
		packet_decoder.cc \
		particle_engine.cc \
		phase_distortion_engine.cc \
		plaits_test.cc \
		random.cc \
		resonator.cc \
		resources.cc \
		sam_speech_synth.cc \
		six_op_engine.cc \
		snare_drum_engine.cc \
		speech_engine.cc \
		string.cc \
		string_engine.cc \
		string_machine_engine.cc \
		string_voice.cc \
		swarm_engine.cc \
		units.cc \
		user_data_receiver.cc \
		virtual_analog_engine.cc \
		virtual_analog_vcf_engine.cc \
		voice.cc \
		waveshaping_engine.cc \
		wavetable_engine.cc \
		wave_terrain_engine.cc
OBJ_FILES      = $(CC_FILES:.cc=.o)
OBJS           = $(patsubst %,$(BUILD_DIR)%,$(OBJ_FILES)) $(STARTUP_OBJ)
DEPS           = $(OBJS:.o=.d)
DEP_FILE       = $(BUILD_DIR)depends.mk

all:  plaits_test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)%.o: %.cc
	g++ -c -DTEST -g -Wall -Werror -msse2 -Wno-unused-variable -Wno-unused-local-typedef -O2 -I. $< -o $@

$(BUILD_DIR)%.d: %.cc
	g++ -MM -DTEST -I. $< -MF $@ -MT $(@:.d=.o)

plaits_test:  $(OBJS)
	g++ -g -o $(TARGET) $(OBJS) -Wl,-no_pie -lm -L/opt/local/lib

depends:  $(DEPS)
	cat $(DEPS) > $(DEP_FILE)

$(DEP_FILE):  $(BUILD_DIR) $(DEPS)
	cat $(DEPS) > $(DEP_FILE)

profile:	plaits_test
	env CPUPROFILE_FREQUENCY=1000 CPUPROFILE=$(BUILD_DIR)/plaits.prof ./plaits_test && pprof --pdf ./plaits_test $(BUILD_DIR)/plaits.prof > profile.pdf && open profile.pdf
	
clean:
	rm $(BUILD_DIR)*.*

include $(DEP_FILE)

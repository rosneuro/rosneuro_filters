#include <stdio.h>
#include <rtfilter.h>
#include <rtf_common.h>

int main(int argc, char* argv[]) {

	constexpr int samplerate 	= 512;
	constexpr int lp_type 		= 0;
	constexpr int hp_type 		= 1;
	constexpr float lp_cutoff 	= 10;
	constexpr float hp_cutoff 	= 1;
	constexpr int lp_order 		= 4;
	constexpr int hp_order 		= 4;

	unsigned int nchannels = 1;
	float lp_fc, hp_fc;

	lp_fc = (1.0f*lp_cutoff) / samplerate;
	hp_fc = (1.0f*hp_cutoff) / samplerate;

	hfilter lp = nullptr;
	hfilter hp = nullptr;

	printf("Setup low-pass filter:\n");
	lp = rtf_create_butterworth(nchannels, RTF_FLOAT, lp_fc, lp_order, lp_type);
	
	printf("Setup high-pass filter:\n");
	hp = rtf_create_butterworth(nchannels, RTF_FLOAT, hp_fc, hp_order, hp_type);


	return 0;
}

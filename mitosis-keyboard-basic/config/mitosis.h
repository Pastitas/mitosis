#ifdef COMPILE_LEFT
	#define PIPE_NUMBER 0
	#define S01 7
	#define S02 4
	#define S03 30
	#define S04 24
	#define S05 28
	#define S06 8
	#define S07 5
	#define S08 2
	#define S09 1
	#define S10 29
	#define S11 9
	#define S12 6
	#define S13 3
	#define S14 0
	#define S15 21
	#define S16 16
	#define S17 13
	#define S18 14
	#define S19 10
	#define S20 15
	#define S21 17
	#define S22 18
	#define S23 19
#endif

#ifdef COMPILE_RIGHT
	#define PIPE_NUMBER 1
	#define S01 2
	#define S02 5
	#define S03 10
	#define S04 15
	#define S05 14
	#define S06 1
	#define S07 4
	#define S08 7
	#define S09 8
	#define S10 13
	#define S11 0
	#define S12 3
	#define S13 6
	#define S14 9
	#define S15 19
	#define S16 25
	#define S17 29
	#define S18 28
	#define S19 30
	#define S20 24
	#define S21 23
	#define S22 22
	#define S23 21
#endif

#define INPUT_MASK (1<<S01 | \
					1<<S02 | \
					1<<S03 | \
					1<<S04 | \
					1<<S05 | \
					1<<S06 | \
					1<<S07 | \
					1<<S08 | \
					1<<S09 | \
					1<<S10 | \
					1<<S11 | \
					1<<S12 | \
					1<<S13 | \
					1<<S14 | \
					1<<S15 | \
					1<<S16 | \
					1<<S17 | \
					1<<S18 | \
					1<<S19 | \
					1<<S20 | \
					1<<S21 | \
					1<<S22 | \
					1<<S23)

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

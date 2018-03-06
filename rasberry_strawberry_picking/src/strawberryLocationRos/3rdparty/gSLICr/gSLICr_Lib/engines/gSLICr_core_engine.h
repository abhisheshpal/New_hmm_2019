// Copyright 2014-2015 Isis Innovation Limited and the authors of gSLICr

#pragma once
#include "gSLICr_seg_engine_GPU.h"


namespace gSLICr
{
	namespace engines
	{
		class core_engine
		{
		public:

			seg_engine* slic_seg_engine;

		public:

			core_engine(const objects::settings& in_settings);
			~core_engine();

			// Function to segment in_img
			void Process_Frame(UChar4Image* in_img);

			// Function to push the EM results to gpu.
			void Push_Cluster_Data(UChar4Image* clusters);

			// Function to get the pointer to the segmented mask image
			const IntImage * Get_Seg_Res();

			// Function to draw segmentation result on out_img
			void Draw_Segmentation_Result(UChar4Image* out_img);

			// Function to draw the black and white contours image.
			void Draw_Bw_Contour(IntImage* bw_contour, int apple_id, bool flag_for_mixing);

			// Write the segmentation result to a PGM image
			void Write_Seg_Res_To_PGM(const char* fileName);
		};
	}
}

// Copyright 2014-2015 Isis Innovation Limited and the authors of gSLICr

#pragma once
#include "../gSLICr_defines.h"

namespace gSLICr
{
	namespace objects
	{
		struct spixel_info
		{
			Vector2f center;
			Vector4f color_info;
			Vector2f red_intensity;
			int id;
			int no_pixels;
			int cluster_id;
		};
	}

	typedef ORUtils::Image<objects::spixel_info> SpixelMap;
}

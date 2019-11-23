#pragma once
#include "Line.h"

struct SeenLine
{
	bool added = false;
	Mat matrix;
	Point point_1;
	Point point_2;
	const Line *line = nullptr;
};
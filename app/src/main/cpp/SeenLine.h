#ifndef SEENLINE_H
#define SEENLINE_H
#include "Line.h"

struct SeenLine
{
	bool added = false;
	Point point_1;
	Point point_2;
	const Line *line = nullptr;
};
#endif
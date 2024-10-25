#pragma once

#include "argb_image.h"
#include "pixel_char.h"


#ifndef ALIGNMENTS
#define ALIGNMENTS

#define ALIGNMENT_LEFT   0
#define ALIGNMENT_RIGHT  1
#define ALIGNMENT_TOP    2
#define ALIGNMENT_BOTTOM 3
#define ALIGNMENT_MIDDLE 5

#endif // !ALIGNMENTS

enum menu_item_type {
	MENU_ITEM_LABEL,
	MENU_ITEM_IMAGE,
};

struct menu_label {
	int menu_item_type;
	int z;
	int x;
	int y;
	int alignment_x;
	int alignment_y;
	int text_alignment_x;
	int text_alignment_y;
	int max_width;
	int max_rows;
	int selectable;
	int hoverable;
	int text_size;
	struct pixel_char text[];
};

struct menu_image {
	int menu_item_type;
	int z;
	int x;
	int y;
	int alignment_x;
	int alignment_y;
	int image_alignment_x;
	int image_alignment_y;
	int hoverable;
	int image;
	int hover_image;
	int image_scalar;
};

struct menu_line {
	int x0;
	int y0;
	int x1;
	int y1;
	float thickness;
	unsigned int color;
	int alignment_x;
	int alignment_y;
};

struct menu_scene {
	int current_item;

	int select_label;
	int select_begin;
	int select_end;
	int selecting;

	int current_pos;

	int image_pos_x;
	int image_pox_y;
	int image_index;

	int items_count;
	int* items[];
};

void menu_scene_draw_line(struct menu_line* line, unsigned int* screen, int width, int height, int scale);

void menu_scene_draw_arrow(struct menu_line* line, unsigned int* screen, int width, int height, int scale);

void menu_scene_draw_label(struct menu_label* label, unsigned int* screen, int width, int height, int scale, const void** resource_map);
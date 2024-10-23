#include "menu.h"

#include "stdio.h"
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <math.h>

#ifndef min
#define min(a, b) (a < b ? a : b)
#endif // !min

#ifndef max
#define max(a, b) (a > b ? a : b)
#endif // !max


int menu_x(int menu_x, int menu_alignment, int menu_width, int scale) {

	int x = menu_x * scale;

	if (menu_alignment == ALIGNMENT_LEFT) x += 0;
	else if (menu_alignment == ALIGNMENT_RIGHT) x += menu_width;
	else if (menu_alignment == ALIGNMENT_MIDDLE) x += menu_width / 2;

	return x;
}

int menu_y(int menu_y, int menu_alignment, int menu_height, int scale) {

	int y = menu_y * scale;

	if (menu_alignment == ALIGNMENT_TOP) y += 0;
	else if (menu_alignment == ALIGNMENT_BOTTOM) y += menu_height;
	else if (menu_alignment == ALIGNMENT_MIDDLE) y += menu_height / 2;

	return y;
}


float point_line_distance(int x0, int y0, int x1, int y1, int px, int py) {
    float dx = (float)(x1 - x0);
    float dy = (float)(y1 - y0);
    float num = fabsf(dy * px - dx * py + x1 * y0 - y1 * x0);
    float denom = sqrtf(dx * dx + dy * dy);
    return num / denom;
}

// The function to draw the line with thickness
int draw_line(unsigned int* screen_pixels, int screen_width, int screen_height, 
              int x0, int y0, int x1, int y1, float thickness, unsigned int color) {

    // Determine the bounding box for the line
    int min_x = fmin(x0, x1) - (int)thickness;
    int max_x = fmax(x0, x1) + (int)thickness;
    int min_y = fmin(y0, y1) - (int)thickness;
    int max_y = fmax(y0, y1) + (int)thickness;

    // Clamp the bounding box to the screen dimensions
    min_x = fmax(min_x, 0);
    max_x = fmin(max_x, screen_width - 1);
    min_y = fmax(min_y, 0);
    max_y = fmin(max_y, screen_height - 1);

    // Loop through the bounding box to fill the line
    float half_thickness = thickness / 2.0f;
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            // Check if this pixel is within the line's thickness
            if (point_line_distance(x0, y0, x1, y1, x, y) <= half_thickness) {
                screen_pixels[y * screen_width + x] = color; // Set the pixel to the line color
            }
        }
    }

    return 0; // Success
}



struct menu_item_wrapper {
	int* item;
	int index;
};

int compare(const void* a, const void* b) {
	return (((struct menu_item_wrapper*)a)->item[1] - ((struct menu_item_wrapper*)b)->item[1]);
}

void menu_scene_draw_line(struct menu_line* line, unsigned int* screen, int width, int height, int scale) {

	draw_line(
		screen, 
		width, 
		height, 
		menu_x(line->x0, line->alignment_x, width, scale), 
		menu_y(line->y0, line->alignment_y, height, scale),
		menu_x(line->x1, line->alignment_x, width, scale), 
		menu_y(line->y1, line->alignment_y, height, scale),
		line->thickness, 
		line->color
	);

}

void menu_scene_draw_label(struct menu_label* label, unsigned int* screen, int width, int height, int scale, const void** resource_map) {

	pixel_char_print_string(
		label->text, 
		label->text_size * scale, 
		2, 
		menu_x(label->x, label->alignment_x, width, scale), 
		menu_y(label->y, label->alignment_y, height, scale), 
		label->text_alignment_x, 
		label->text_alignment_y, 
		label->max_width,
		label->max_rows,
		screen, 
		width, 
		height, 
		resource_map
	);
}

void menu_scene_frame(struct menu_scene* menu, unsigned int* screen, int width, int height, int scale, const void** resource_map, int mouse_x, int mouse_y, int mouse_left_click) {

	union argb_pixel* screen_argb = (union argb_pixel*)screen;


	struct menu_item_wrapper* ordered_menu_items = (struct menu_item_wrapper*)alloca(sizeof(struct menu_item_wrapper) * menu->items_count);;
	for (int i = 0; i < menu->items_count; i++) { ordered_menu_items[i].index = i; ordered_menu_items[i].item = menu->items[i]; }

	qsort(ordered_menu_items, menu->items_count, sizeof(struct menu_item_wrapper), compare);

	for (int i = 0; i < menu->items_count; i++) {
		switch (ordered_menu_items[i].item[0]) {

		case MENU_ITEM_LABEL: {

			struct menu_label* label = (struct menu_label*)ordered_menu_items[i].item;
			
			int string_size = 1;
			for (; label->text[string_size - 1].value != '\0'; string_size++);

			struct pixel_char* text = (struct pixel_char*)alloca(sizeof(struct pixel_char) * string_size);
			memcpy(text, label->text, sizeof(struct pixel_char) * string_size);

			if (label->hoverable) {
				int select_index = pixel_char_get_hover_index(
					text,
					label->text_size * scale,
					2,
					menu_x(label->x, label->alignment_x, width, scale),
					menu_y(label->y, label->alignment_y, height, scale),
					label->text_alignment_x,
					label->text_alignment_y,
					label->max_width,
					label->max_rows,
					resource_map,
					mouse_x,
					mouse_y
				);

				if (select_index >= 0) {
					menu->current_item = ordered_menu_items[i].index;
					menu->current_pos = select_index;
				}

				if (label->selectable) {

					if (mouse_left_click == 0b11) {
						if (menu->select_label == i && menu->select_begin == select_index && menu->selecting == 1) {

							for (menu->select_end = select_index; ; menu->select_end++) if (
								(text[menu->select_end].value < 'a' || text[menu->select_end].value > 'z') &&
								(text[menu->select_end].value < 'A' || text[menu->select_end].value > 'Z') &&
								(text[menu->select_end].value < '0' || text[menu->select_end].value > '9')
								) break;

							if (menu->select_end != select_index) menu->select_end--;

							for (menu->select_begin = select_index; menu->select_begin > 0; menu->select_begin--) if (
								(text[menu->select_begin - 1].value < 'a' || text[menu->select_begin - 1].value > 'z') &&
								(text[menu->select_begin - 1].value < 'A' || text[menu->select_begin - 1].value > 'Z') &&
								(text[menu->select_begin - 1].value < '0' || text[menu->select_begin - 1].value > '9')
								) break;

							menu->selecting = 0;
						}
						else {
							menu->select_label = i;
							menu->select_begin = select_index;
							menu->select_end = -1;
							menu->selecting = 1;
						}

					}

					else if (mouse_left_click && menu->select_label == i && select_index != -1 && (menu->select_begin != select_index || menu->select_end != -1) && menu->selecting == 1) {
						menu->select_end = select_index;
					}

					if (menu->select_begin >= 0 && menu->select_end >= 0) {
						for (int j = min(menu->select_begin, menu->select_end); j <= max(menu->select_begin, menu->select_end) && j > 0 && j < string_size; j++) {
							text[j].color = 0xffffff00;
							text[j].masks |= PIXEL_CHAR_BACKGROUND_MASK;
						}
					}
				}
			}

			pixel_char_print_string(
				text, 
				label->text_size * scale, 
				2, 
				menu_x(label->x, label->alignment_x, width, scale), 
				menu_y(label->y, label->alignment_y, height, scale), 
				label->text_alignment_x, 
				label->text_alignment_y, 
				label->max_width,
				label->max_rows,
				screen, 
				width, 
				height, 
				resource_map
			);

			break;
		}

		case MENU_ITEM_IMAGE: {

			struct menu_image* image = (struct menu_image*)ordered_menu_items[i].item;

			int image_to_draw = image->image;

			int x_rel = 0;
			int y_rel = 0;

			int image_index = argb_image_get_rel_position((struct argb_image*)resource_map[image->image],
				menu_x(image->x, image->alignment_x, width, scale),
				menu_y(image->y, image->alignment_y, height, scale),
				image->image_alignment_x,
				image->image_alignment_y,
				0,
				0,
				width,
				height,
				image->image_scalar * scale,
				&x_rel,
				&y_rel,
				mouse_x,
				mouse_y
			);

			if (image_index >= 0) {
				menu->current_item = ordered_menu_items[i].index;
				menu->image_index = image_index;
				menu->image_pos_x = x_rel;
				menu->image_pox_y = y_rel;
				if(image->hoverable) image_to_draw = image->hover_image;
			}

			argb_image_draw(
				MODE_RGB,
				(struct argb_image*)resource_map[image_to_draw],
				menu_x(image->x, image->alignment_x, width, scale),
				menu_y(image->y, image->alignment_y, height, scale),
				image->image_alignment_x, 
				image->image_alignment_y, 
				0, 
				0, 
				screen, 
				width, 
				height, 
				image->image_scalar * scale
			);

			break;
		}

		default:
			break;
		}
	}

}
#include "1028A/init.h"

using namespace _1028A;

Gif::Gif(const char* fname, lv_obj_t* parent) {
	FILE* fp = fopen(fname, "rb");

	if(fp != NULL) {
		fseek(fp, 0, SEEK_END);
		size_t len = ftell(fp);
		fseek(fp, 0, SEEK_SET);

		_gifmem = malloc(len);

		if(_gifmem != NULL) {
			fread(_gifmem, 1, len, fp);
		} else {
			std::cerr << "Gif::Gif - not enough memory for gif file" << std::endl;
		}
		fclose(fp);

		if(_gifmem != NULL) {
			FILE* memfp = fmemopen(_gifmem, len, "rb");

			_gif = gd_open_gif(memfp);
			if(_gif == NULL) {
				std::cerr << "Gif::Gif - error opening \"" + std::string(fname) + "\"" << std::endl;
				_cleanup();
				return;
			}

			_buffer = (uint8_t*)malloc(_gif->width * _gif->height * BYTES_PER_PIXEL);
			if(_buffer == NULL) {
				_cleanup();
				std::cerr << "Gif::Gif - not enough memory for frame buffer" << std::endl;
			} else {
				_cbuf = new lv_color_t[_gif->width * _gif->height];
				_canvas = lv_canvas_create(parent, NULL);
				lv_canvas_set_buffer(_canvas, _cbuf, _gif->width, _gif->height, LV_IMG_CF_TRUE_COLOR_ALPHA);
				_task = pros::c::task_create(_render_task, static_cast<void*>(this), TASK_PRIORITY_DEFAULT-2, TASK_STACK_DEPTH_DEFAULT, ("GIF - \"" + std::string(fname) + "\"").c_str());
			}
		}
	} else {
		std::cerr << "Gif::Gif - unable to open \"" + std::string(fname) + "\" (file not found)" << std::endl;
	}
};


Gif::~Gif() {
	_cleanup();
}

void Gif::pause(){
	pros::c::task_suspend(_task);
}

void Gif::resume(){
	pros::c::task_resume(_task);
}

void Gif::clean() {
	_cleanup();
}

void Gif::_cleanup() {
	if(_canvas) { lv_obj_del(_canvas); _canvas = nullptr; }
	if(_cbuf) { delete[] _cbuf; _cbuf = nullptr; }
	if(_buffer) { free(_buffer); _buffer = nullptr; }
	if(_gif) { gd_close_gif(_gif); _gif = nullptr; }
	if(_gifmem) { free(_gifmem); _gifmem = nullptr; }
	// deleting task kills this thread
	if(_task) { pros::c::task_delete(_task); _task = nullptr; }
}


void Gif::_render() {

	for (size_t looped = 1;; looped++) {
		while (gd_get_frame(_gif)) {
			int32_t now = pros::millis();

			gd_render_frame(_gif, _buffer);

			for (size_t i = 0; i < _gif->height * _gif->width; i++) {
				_cbuf[i].red = _buffer[(i * BYTES_PER_PIXEL)];
				_cbuf[i].green = _buffer[(i * BYTES_PER_PIXEL) + 1];
				_cbuf[i].blue = _buffer[(i * BYTES_PER_PIXEL) + 2];
				_cbuf[i].alpha = _buffer[(i * BYTES_PER_PIXEL) + 3];
			};

			lv_obj_invalidate(_canvas);

			int32_t delay = _gif->gce.delay * 10;
			int32_t delta = pros::millis() - now;
			delay -= delta;

			if(delay > 0) pros::delay(delay);
		}

		if(looped == _gif->loop_count) break;
		gd_rewind(_gif);
	}

	_cleanup();
}


void Gif::_render_task(void* arg) {
	Gif* instance = static_cast<Gif*>(arg);
	instance->_render();
}
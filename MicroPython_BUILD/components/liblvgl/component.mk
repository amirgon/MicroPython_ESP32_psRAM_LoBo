#
# Component Makefile
#



COMPONENT_SRCDIRS := . \
	drv \
	lvgl/lv_core \
	lvgl/lv_draw \
	lvgl/lv_objx \
	lvgl/lv_hal \
	lvgl/lv_misc \
	lvgl/lv_fonts \
	lvgl/lv_themes
	
COMPONENT_ADD_INCLUDEDIRS := $(COMPONENT_SRCDIRS) .

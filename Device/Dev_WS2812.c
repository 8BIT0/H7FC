#include "Dev_WS2812.h"

/* external function */
static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj);
static bool Dev_WS2812_Write(DevWS2812Obj_TypeDef *p_obj, uint8_t index, RGB_TypeDef rgb);
static bool Dev_WS2812_Trans(DevWS2812Obj_TypeDef *p_obj);

/* internal function */
static bool Dev_WS2812_Set_Bright(DevWS2812Obj_TypeDef *p_obj);
static float find_min(float a, float b, float c);
static float find_max(float a, float b, float c);
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float  *h, float  *s, float  *v);
void hsv2rgb(float  h, float  s, float  v, uint8_t *r, uint8_t *g, uint8_t *b);

DevWS2812_TypeDef DevWS2812 = {
    .init = Dev_WS2812_Init,
    .set = Dev_WS2812_Write,
    .trans = Dev_WS2812_Trans,
};

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj)
{
    if ((p_obj == NULL) || \
        (p_obj->led_num == NULL) || \
        (p_obj->port_init == NULL) || \
        (p_obj->port_send == NULL) || \
        (p_obj->p_malloc == NULL) || \
        (p_obj->p_free == NULL))
        return false;

    p_obj->buff = p_obj->p_malloc(sizeof(WS2812_CtlData_TypeDef) * p_obj->led_num);
    if ((p_obj->buff == NULL) || !p_obj->port_init(p_obj))
    {
        p_obj->p_free(p_obj->buff);
        return false;
    }

    /* set default color */
    p_obj->RGB = WS2812_GHOSTWHITE;

    return true;
}

static bool Dev_WS2812_Write(DevWS2812Obj_TypeDef *p_obj, uint8_t index, RGB_TypeDef rgb)
{
    uint8_t data[3] = {0};
    uint8_t bit = 0x00;
    float bright_pct = 0.0f;
    HSV_TypeDef HSV;

    if ((p_obj == NULL) || \
        (p_obj->port_send == NULL) || \
        (p_obj->buff == NULL) || \
        (index >= p_obj->led_num))
        return false;

    p_obj->RGB = rgb;
    bright_pct = ((float)p_obj->RGB.bright / UINT8_MAX);

    if (!Dev_WS2812_Set_Bright(p_obj))
        return false;

    data[0] = p_obj->RGB.G;
    data[1] = p_obj->RGB.R;
    data[2] = p_obj->RGB.B;

    /* convert to HSV */
    rgb2hsv(p_obj->RGB.R, p_obj->RGB.G, p_obj->RGB.B, &HSV.H, &HSV.S, &HSV.V);
    HSV.V = bright_pct;
    hsv2rgb(HSV.H, HSV.S, HSV.V, &p_obj->RGB.R, &p_obj->RGB.G, &p_obj->RGB.B);

    p_obj->buff[index].ctl_data[0] = 0;
    p_obj->buff[index].ctl_data[1] = 0;
    p_obj->buff[index].ctl_data[2] = 0;
    p_obj->buff[index].ctl_data[3] = 0;
    
    p_obj->buff[index].ctl_data[WS2812_DATA_SIZE - 1] = 0;
    p_obj->buff[index].ctl_data[WS2812_DATA_SIZE - 2] = 0;
    p_obj->buff[index].ctl_data[WS2812_DATA_SIZE - 3] = 0;
    p_obj->buff[index].ctl_data[WS2812_DATA_SIZE - 4] = 0;

    for (uint8_t i = 4; i < WS2812_VALID_DATA_SIZE + 4; i ++)
    {
        bit = 0x00;
        p_obj->buff[index].ctl_data[i] = WS2812_T0H;
        bit |= 1 << (7 - (i % 8));
        if (data[i / 8] & bit)
            p_obj->buff[index].ctl_data[i] = WS2812_T1H;
    }

    return true;
}

static bool Dev_WS2812_Trans(DevWS2812Obj_TypeDef *p_obj)
{
    if ((p_obj == NULL) || \
        (p_obj->led_num == NULL) || \
        (p_obj->buff == NULL) || \
        (p_obj->port_send == NULL))
        return false;

    return p_obj->port_send(p_obj->port_Obj);
}

static bool Dev_WS2812_Set_Bright(DevWS2812Obj_TypeDef *p_obj)
{
    if (p_obj == NULL)
        return false;

    return true;
}

static float find_min(float a, float b, float c)
{
	float m;
	
	m = a < b ? a : b;
	return (m < c ? m : c); 
}

static float find_max(float a, float b, float c)
{
	float m;
	
	m = a > b ? a : b;
	return (m > c ? m : c); 
}
  
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float  *h, float  *s, float  *v)
{
	float  red, green ,blue;
	float  cmax, cmin, delta;
	
	red = (float)r / UINT8_MAX;
	green = (float)g / UINT8_MAX;
	blue = (float)b / UINT8_MAX;
	
	cmax = find_max(red, green, blue);
	cmin = find_min(red, green, blue);
	delta = cmax - cmin;
	
	/* H */
	if (delta == 0)
	{
		*h = 0;
	}
	else
	{
		if (cmax == red)
		{
            *h = 60 * ((green - blue) / delta) + 360;
			if (green >= blue)
				*h = 60 * ((green - blue) / delta);
		}
		else if (cmax == green)
		{
			*h = 60 * ((blue - red) / delta + 2);
		}
		else if (cmax == blue)
		{
			*h = 60 * ((red - green) / delta + 4);
		}
	}
	
	/* S */
    *s = delta / cmax;
	if (cmax == 0)
		*s = 0;
	
	/* V */
	*v = cmax;
}

static void cnv2rgb(float gain1, float gain2, float gain3, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = UINT8_MAX * gain1;
    *g = UINT8_MAX * gain2;
    *b = UINT8_MAX * gain3;
}

void hsv2rgb(float  h, float  s, float  v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int  hi = ((int)h / 60) % 6;
    float  f = h * 1.0 / 60 - hi;
    float  p = v * (1 - s);
    float  q = v * (1 - f * s);
    float  t = v * (1- (1 - f) * s);

    switch (hi)
	{
        case 0: cnv2rgb(v, t, p, r, g, b); break;
        case 1: cnv2rgb(q, v, p, r, g, b); break;
        case 2: cnv2rgb(p, v, t, r, g, b); break;
        case 3: cnv2rgb(p, q, v, r, g, b); break;
        case 4: cnv2rgb(t, p, v, r, g, b); break;
        case 5: cnv2rgb(v, p, q, r, g, b); break;
        default: break;
    }
}



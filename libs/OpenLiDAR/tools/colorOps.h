#pragma once

inline double map(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
    double outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
    return outVal;
}

inline float saturate(float value) { 
    return std::max (0.0f, std::min (1.0f, value)); 
}

inline void hue2rgb(float _hue, unsigned char& _r, unsigned char& _g, unsigned char& _b ) {
    float r = saturate( abs( fmod( _hue * 6., 6.) - 3.) - 1. );
    float g = saturate( abs( fmod( _hue * 6. + 4., 6.) - 3.) - 1. );
    float b = saturate( abs( fmod( _hue * 6. + 2., 6.) - 3.) - 1. );

    #ifdef HSV2RGB_SMOOTH
    r = r*r*(3. - 2. * r);
    g = g*g*(3. - 2. * g);
    b = b*b*(3. - 2. * b);
    #endif

    _r = static_cast<unsigned char> (pcl_lrint(r * 255));
    _g = static_cast<unsigned char> (pcl_lrint(g * 255)); 
    _b = static_cast<unsigned char> (pcl_lrint(b * 255));
}

inline unsigned char* toRGB(const float* float_image, 
                            int width, int height, 
                            float min_value=std::numeric_limits<float>::infinity(), float max_value=std::numeric_limits<float>::infinity())
{
    int size = width*height;
    int arraySize = 3 * size;
    unsigned char* data = new unsigned char[arraySize];
    unsigned char* dataPtr = data;
  
    bool recalculateMinValue = std::isinf (min_value),
         recalculateMaxValue = std::isinf (max_value);
    if (recalculateMinValue) min_value = std::numeric_limits<float>::infinity();
    if (recalculateMaxValue) max_value = -std::numeric_limits<float>::infinity();
  
    if (recalculateMinValue || recalculateMaxValue) {
        for (int i=0; i<size; ++i) {
            float value = float_image[i];
            if (!std::isfinite(value)) continue;
            if (recalculateMinValue)  min_value = (std::min)(min_value, value);
            if (recalculateMaxValue)  max_value = (std::max)(max_value, value);
        }
    }
    
    float factor = 1.0f / (max_value-min_value), 
          offset = -min_value;
  
    for (int i=0; i<size; ++i) {
        unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
        float value = float_image[i];
    
        if ( !std::isfinite(value) ) {
            hue2rgb(value, r, g, b);
            continue;
        }
    
        // Normalize value to [0, 1]
        value = saturate( factor * (value + offset) );
        hue2rgb(value, r, g, b);
    }
  
    return data;
}
#ifndef SVG_FAVICON_H
#define SVG_FAVICON_H

#include <Arduino.h>

const char SVG_FAVICON[] PROGMEM = R"=====(
<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>
<circle cx='50' cy='50' r='48' fill='#f4f4f4'/>
<circle cx='50' cy='50' r='42' fill='none' stroke='#0056b3' stroke-width='6'/>
<path d='M50 50 L50 18' stroke='#dc3545' stroke-width='6' stroke-linecap='round' transform='rotate(45 50 50)'/>
<circle cx='50' cy='50' r='8' fill='#0056b3'/>
<path d='M15 65 Q50 50 85 65' fill='none' stroke='#333' stroke-width='4'/>
<path d='M20 70 L15 65 L20 60' fill='#dc3545' stroke='#dc3545' stroke-width='3'/>
<path d='M80 70 L85 65 L80 60' fill='#28a745' stroke='#28a745' stroke-width='3'/>
</svg>
)=====";

#endif // SVG_FAVICON_H

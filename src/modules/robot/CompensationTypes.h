/*
This file is part of Carvera Community Firmware, a fork of Smoothieware ([http://smoothieware.org/](http://smoothieware.org/)).
Carvera Community Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Carvera Community Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Carvera Community Firmware. If not, see [http://www.gnu.org/licenses/](http://www.gnu.org/licenses/).
*/

#ifndef COMPENSATION_TYPES_H
#define COMPENSATION_TYPES_H

enum class CompensationType {
    NONE = 0,   // G40 - canel compensation
    LEFT = 1,   // G41 - Tool offset left
    RIGHT = 2   // G42 - Tool offset right
};

#endif // COMPENSATION_TYPES_H

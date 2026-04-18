/*
      This file is part of Smoothie (http://smoothieware.org/).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef COMPENSATION_TYPES_H
#define COMPENSATION_TYPES_H

/**
 * Cutter Compensation Types
 * Corresponds to G-code commands:
 * - G40: Compensation Off
 * - G41: Compensation Left (tool offset to left of path)
 * - G42: Compensation Right (tool offset to right of path)
 */
enum class CompensationType {
    NONE = 0,   // G40 - No compensation
    LEFT = 1,   // G41 - Tool offset to left
    RIGHT = 2   // G42 - Tool offset to right
};

#endif // COMPENSATION_TYPES_H

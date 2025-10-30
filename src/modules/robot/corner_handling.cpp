#include "CompensationPreprocessor.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include <cmath>

void CompensationPreprocessor::calculate_corner(const Move& prev, const Move& current, const Move& next, float* output) {
    // Calculate unit vectors for both moves
    float dx1 = current.end[0] - current.start[0];
    float dy1 = current.end[1] - current.start[1];
    float len1 = hypotf(dx1, dy1);
    
    float dx2 = next.end[0] - next.start[0];
    float dy2 = next.end[1] - next.start[1];
    float len2 = hypotf(dx2, dy2);
    
    if (len1 < 0.00001F || len2 < 0.00001F) {
        output[0] = current.end[0];
        output[1] = current.end[1];
        return;
    }
    
    // Calculate dot product to find angle between moves
    float ux1 = dx1/len1, uy1 = dy1/len1;
    float ux2 = dx2/len2, uy2 = dy2/len2;
    float dot = ux1*ux2 + uy1*uy2;
    
    if (fabsf(dot) > 0.99999F) {
        // Lines are nearly parallel, use simple perpendicular offset
        calculate_line_offset(current, output);
        return;
    }
    
    // Determine if this is an inside or outside corner
    bool inside = is_inside_corner(prev, next);
    
    // Calculate the offset at the corner
    float angle = acosf(dot);
    float offset = comp_radius;
    
    if (inside) {
        // For inside corners, we need to extend the offset
        offset /= sinf(angle/2.0F);
        
        // Check if the corner is too tight
        if (offset > comp_radius * 3.0F) {
            THEKERNEL->streams->printf("DBG:CompPrep: Corner angle too tight, limiting offset\n");
            offset = comp_radius * 3.0F;
        }
    }
    
    // Calculate intersection of offset lines
    calculate_intersection(prev, next, output);
    
    THEKERNEL->streams->printf("DBG:CompPrep: Corner processed angle=%.1f deg %s offset=%.3f\n",
        angle * 180.0F / 3.14159F,
        inside ? "inside" : "outside",
        offset);
}

bool CompensationPreprocessor::is_inside_corner(const Move& prev, const Move& next) const {
    // Calculate cross product of move vectors
    float dx1 = next.end[0] - next.start[0];
    float dy1 = next.end[1] - next.start[1];
    float dx2 = prev.end[0] - prev.start[0];
    float dy2 = prev.end[1] - prev.start[1];
    
    float cross = dx1*dy2 - dy1*dx2;
     return (cross > 0) == (comp_side == Compensation::LEFT);
}

void CompensationPreprocessor::calculate_intersection(const Move& line1, const Move& line2, float* output) {
    // Get vectors for each line
    float dx1 = line1.end[0] - line1.start[0];
    float dy1 = line1.end[1] - line1.start[1];
    float dx2 = line2.end[0] - line2.start[0];
    float dy2 = line2.end[1] - line2.start[1];
    
    // Calculate offset vectors (perpendicular)
     float scale = (comp_side == Compensation::LEFT) ? comp_radius : -comp_radius;
    float ox1 = -dy1 * scale;
    float oy1 = dx1 * scale;
    float ox2 = -dy2 * scale;
    float oy2 = dx2 * scale;
    
    // Calculate offset line endpoints
    float p1x = line1.end[0] + ox1;
    float p1y = line1.end[1] + oy1;
    float p2x = line2.start[0] + ox2;
    float p2y = line2.start[1] + oy2;
    
    // Find intersection of offset lines
    float det = dx1*dy2 - dy1*dx2;
    if (fabsf(det) < 0.00001F) {
        // Lines are parallel, use midpoint
        output[0] = (p1x + p2x) * 0.5F;
        output[1] = (p1y + p2y) * 0.5F;
        return;
    }
    
    float t = ((p2x - p1x)*dy2 - (p2y - p1y)*dx2) / det;
    output[0] = p1x + dx1 * t;
    output[1] = p1y + dy1 * t;
}
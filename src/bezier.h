/*
 * bezier.h
 *
 * simple bezier curve library adapted from http://www.cubic.org/docs/bezier.htm 
 * without external dependencies
 *
 * Structure is adapted from spline.h available from http://kluge.in-chemnitz.de/opensource/spline/
 */

#include <cstdio>


// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files
namespace
{

    namespace bz
    {
        
        // spline interpolation
        class bezier
        {
        public:

        private:
            
            struct point
                {
                    float x;
                    float y;
                };

        public:
            // function definitions
            // empty constructor
            bezier();
            // simple linear interpolation between two points
            void lerp(point& dest, const point& a, const point& b, const float t);
            // evaluate a point on a bezier-curve. t goes from 0 to 1.0
            void bezier_calc(point &dest, const point& a, const point& b, const point& c, const point& d, const float t);
            
        };
        
        // -----------------------
        // bezier implementation
        // -----------------------

        // simple linear interpolation between two points
        void bezier::lerp(bezier::point& dest, const bezier::point& a, const bezier::point& b, const float t)
        {
            dest.x = a.x + (b.x-a.x)*t;
            dest.y = a.y + (b.y-a.y)*t;
        }

        // evaluate a point on a bezier-curve. t goes from 0 to 1.0
        void bezier::bezier_calc(bezier::point &dest, const bezier::point& a, const bezier::point& b, const bezier::point& c, const bezier::point& d, const float t)
        {
            bezier::point ab,bc,cd,abbc,bccd;               
            bezier::lerp(ab, a,b,t);           // point between a and b (green)
            bezier::lerp(bc, b,c,t);           // point between b and c (green)
            bezier::lerp(cd, c,d,t);           // point between c and d (green)
            bezier::lerp(abbc, ab,bc,t);       // point between ab and bc (blue)
            bezier::lerp(bccd, bc,cd,t);       // point between bc and cd (blue)
            bezier::lerp(dest, abbc,bccd,t);   // point on the bezier-curve (black)
        }

    }
}
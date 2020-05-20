#include <iostream>
#include <fstream>
#include <vector>
#include <future>

#include "primitives.h"
#include "scene_objects.h"
#include "utilities.h"
#include "CImg.h"
#include "profiler.h"

using namespace std;
using namespace Geometry3D;
using namespace cimg_library;

int main () {

    Point camera(0., 0., 0.);

    vector<vector<Point>> matrix(1000);
    for (auto& row : matrix){
        row.resize(1000);
    }

    double y = -499.5, z = -499.5;
    for (size_t i = 0; i < 1000; ++i) {
        for (size_t j = 0; j < 1000; ++j){
            matrix[i][j] = Point(10., y, z);
           y += 1.0;
        }

        z += 1.0;
        y = -249.5;
    }



    Point minPoint(15, -400, -350);
    Point maxPoint(250, 300, 300);
    Box b(minPoint, maxPoint);

    CImg<unsigned char> image(1000, 1000, 1, 3, 0);

    unsigned char color[3] = {0, 255, 0};
    LOG_DURATION("Rendering")
    {
        vector<future<void>> futures_;

        auto f = async ( [&matrix, &b, &camera, &image, &color] {
            for (size_t i = 0; i < 1000; ++i) {
                for (size_t j = 0; j < 1000; ++j) {
                    Vector v = matrix[i][j] - camera;
                    Ray ray(camera, v);

                    if (b.Intersect(ray).has_value()) {
                        // cout << "YES" << endl;
                        image.draw_point(i, j, color);
                    }
                }
            }
        }); /*

        auto g = async ( [&matrix, &b, &camera, &image, &color] {
            for (size_t i = 250; i < 500; ++i) {
                for (size_t j = 0; j < 1000; ++j) {
                    Vector v = matrix[i][j] - camera;
                    Ray ray(camera, v);

                    if (b.Intersect(ray).has_value()) {
                        // cout << "YES" << endl;
                        image.draw_point(i, j, color);
                    }
                }
            }
        });

        auto c = async ( [&matrix, &b, &camera, &image, &color] {
            for (size_t i = 500; i < 750; ++i) {
                for (size_t j = 0; j < 1000; ++j) {
                    Vector v = matrix[i][j] - camera;
                    Ray ray(camera, v);

                    if (b.Intersect(ray).has_value()) {
                        // cout << "YES" << endl;
                        image.draw_point(i, j, color);
                    }
                }
            }
        });

        auto l = async ( [&matrix, &b, &camera, &image, &color] {
            for (size_t i = 750; i < 1000; ++i) {
                for (size_t j = 0; j < 1000; ++j) {
                    Vector v = matrix[i][j] - camera;
                    Ray ray(camera, v);

                    if (b.Intersect(ray).has_value()) {
                        // cout << "YES" << endl;
                        image.draw_point(i, j, color);
                    }
                }
            }
        }); */

        futures_.push_back(move(f)); /* futures_.push_back(move(g));
        futures_.push_back(move(c)); futures_.push_back(move(l)); */


        /* for (size_t i = 0; i < 1000; ++i) {
            for (size_t j = 0; j < 1000; ++j) {
                Vector v = matrix[i][j] - camera;
                Ray ray(camera, v);

                if (RayBox(b, ray).has_value()) {
                    // cout << "YES" << endl;
                    image.draw_point(i, j, color);
                }
            }
        } */



    }

    image.save("test.bmp");

   return 0;
}


//
// Created by Osip Chin on 20.05.2020.
//

#include "screen.h"
#include "utilities.h"
#include "CImg.h"
#include "lighting.h"
#include "profiler.h"

#include <omp.h>
#include <cmath>
#include <fstream>
#include <future>
#include <functional>
#include <vector>

#ifndef PI
#define PI 3.14159265

using namespace Geometry3D;
using namespace cimg_library;

Screen LoadScreenParameters(const std::string& filename) {
    std::ifstream inputFile(filename);
    if (!inputFile) {
        throw std::runtime_error("No input file!");
    }

    Screen newScreen;
    size_t dataCount = 0;

    for (std::string tmp; inputFile >> tmp; ) {
        ++dataCount;
        if (tmp == "camera") {
            inputFile >> newScreen.camera_;
        } else if (tmp == "distance") {
            inputFile >> newScreen.distance_;
        } else if (tmp == "alpha") {
            inputFile >> newScreen.alpha_;
            newScreen.alpha_ = newScreen.alpha_ * PI / 360.;
        } else if (tmp == "normal") {
            inputFile >> newScreen.normal_;
            newScreen.normal_.Normalize();
        } else if (tmp == "yAxis") {
            inputFile >> newScreen.yAxis_;
            newScreen.yAxis_.Normalize();
        } else if (tmp == "dimensions") {
            inputFile >> newScreen.height_ >> newScreen.width_;
        } else if (tmp == "light") {
            inputFile >> newScreen.light_;
        } else {
            throw std::runtime_error("Invalid data format");
        }
    }

    if (dataCount != 7) {
        throw std::runtime_error("Screen parameters missing");
    }
    inputFile.close();

    return newScreen;
}

void RenderSingleThread(const Screen& screen, const std::vector<std::shared_ptr<Geometry3D::Object>>& objects,
                        const Vector& xAxis, double pixel_dimension, const std::vector<RGB>& colors,
                        Point start, size_t y0, size_t h,
                        CImg<unsigned char>& image) {

    std::vector<std::vector<Geometry3D::Point>> matrix(screen.width_);
    for (auto& row : matrix) {
        row.resize(h);
    }

    for (size_t i = y0; i < y0 + h; ++i) {
        for (size_t j = 0; j < screen.width_; ++j) {
            matrix[j][i] = start;
            start = start + (-xAxis * pixel_dimension);
        }
        start = start + (-screen.yAxis_ * pixel_dimension);
        if (i != y0 + h - 1) {
            start = start + (xAxis * (pixel_dimension * screen.width_));
        }
    }

    omp_set_num_threads(4);
    omp_set_dynamic(0);

#pragma omp parallel for shared(screen, objects, colors, image)
    for (size_t i = y0; i < y0 + h; ++i) {
        for (size_t j = 0; j < screen.width_; ++j) {
            Ray curRay(screen.camera_, matrix[j][i]);

            for (size_t k = 0; k < 3; ++k) {
                auto intersection = objects[k]->Intersect(curRay);
                if (intersection.has_value()) {
                    RGB resColor = GetColor(objects[k], screen.light_,
                                            intersection.value(), colors[k]);
                    image.draw_point(j, i, resColor.rgb_);
                    break;
                }
            }
        }
    }
}

void Render(const Screen& screen, const std::vector<std::shared_ptr<Geometry3D::Object>>& objects,
            const std::vector<RGB>& colors) {
    using namespace Geometry3D;

    const Point screenCentre = screen.camera_ + screen.normal_ * screen.distance_;
    const Vector xAxis = CrossProduct(screen.yAxis_, screen.normal_);
    const Point camera = screen.camera_;

    size_t screenHeight = screen.height_,
            screenWidth  = screen.width_;

    // 1) Assuming screenHeight and screenWidth are even numbers
    // 2) Since screenNormal and yAxis are noramlizes => xAxis normalized as well
    // 3) Assuming pixels are square

    double height_half = screen.distance_ * tan(screen.alpha_);
    double pixel_dimension = 2 * height_half / screenHeight;

    Point minScreenPoint = screenCentre +
                           (xAxis * (screenWidth * pixel_dimension / 2.));
    minScreenPoint = minScreenPoint +
                     (screen.yAxis_ * (screenHeight * pixel_dimension/ 2.));

    Point minScreenPixelCentre = minScreenPoint +
                                 (-xAxis * (0.5 * pixel_dimension));
    minScreenPixelCentre = minScreenPixelCentre +
                           (-screen.yAxis_ * (0.5 * pixel_dimension));

    --screenHeight;
    --screenWidth;

    CImg<unsigned char> image(screenWidth, screenHeight, 1, 3, 255);

    /* const size_t NUM_THREADS = 4;
    std::vector<std::future<void>> futures_(NUM_THREADS); */

    LOG_DURATION("MultiThread")
    {
        RenderSingleThread(screen, objects, xAxis, pixel_dimension, colors, minScreenPixelCentre,
                           0, screenHeight, image);
    }

    image.save_bmp("out.bmp");
}

#endif // PI
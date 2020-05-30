//
// Created by Osip Chin on 20.05.2020.
//

#include "screen.h"
#include "utilities.h"
#include "CImg.h"
#include "lighting.h"
#include "profiler.h"

#include <cmath>
#include <fstream>
#include <future>
#include <functional>

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
    for (size_t i = y0; i < y0 + h; ++i) {
        for (size_t j = 0; j < screen.width_; ++j) {
            Ray curRay(screen.camera_, start - screen.camera_);

            for (size_t k = 0; k < 3; ++k) {
                auto intersection = objects[k]->Intersect(curRay);
                if (intersection.has_value()) {
                    RGB resColor = GetColor(objects[k], screen.light_,
                                            intersection.value(), colors[k]);
                    image.draw_point(j, i, resColor.rgb_);
                    break;
                }
            }

            start = start + (-xAxis * pixel_dimension);
        }

        start = start + (-screen.yAxis_ * pixel_dimension);
        if (i != y0 + h - 1) {
            start = start + (xAxis * (pixel_dimension * screen.width_));
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

    const size_t NUM_THREADS = 4;
    std::vector<std::future<void>> futures_(NUM_THREADS);

    LOG_DURATION("MultiThread")
    {
        size_t screen_height_part = (screenHeight + 1) / NUM_THREADS;
        for (size_t i = 0; i < NUM_THREADS; ++i) {
            Point start = minScreenPixelCentre + (-screen.yAxis_ *
                                                  (static_cast<double>(i) * screenHeight * pixel_dimension /
                                                   NUM_THREADS));
            size_t y0 = i * (screenHeight + 1) / 4;
            if (i == NUM_THREADS - 1) {
                --screen_height_part;
            }

            futures_[i] = std::async(RenderSingleThread, std::ref(screen), std::ref(objects),
                                     std::ref(xAxis), pixel_dimension, std::ref(colors),
                                     start, y0, screen_height_part, std::ref(image));
        }

        for (auto &f : futures_) {
            f.get();
        }
    }

    image.save_bmp("out.bmp");
}



#endif // PI
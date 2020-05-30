#include <iostream>
#include <string>
#include <vector>

#include "screen.h"

using namespace Geometry3D;


int main () {
    
   const std::string screenInput  = "screen_input.txt",
                     objectsInput = "objects_input.txt";

   Screen screen;
   std::vector<std::shared_ptr<Object>> objects;
   std::vector<RGB> colors = {
           {255, 0, 0},
           {0, 255, 0},
           {0, 0, 255}
   };

   try {

       screen = LoadScreenParameters(screenInput);
       objects = LoadObjects(objectsInput);

   } catch (const std::exception& e) {
       std::cerr << e.what() << std::endl;
       return -1;
   }

   {
       std::vector<double> distance_to_object(3);
       for (size_t i = 0; i < 3; ++i) {
           distance_to_object[i] = objects[i]->DistanceToPoint(screen.camera_);
       }

       // Bubble sort two vectors
       for (size_t i = 0; i < 3; ++i) {
           for (size_t j = 1; j < 3; ++j) {
               if (distance_to_object[i - 1] > distance_to_object[i]) {
                   std::swap(objects[i - 1], objects[i]);
               }
           }
       }
   }

   Render(screen, objects, colors);

   return 0;
}


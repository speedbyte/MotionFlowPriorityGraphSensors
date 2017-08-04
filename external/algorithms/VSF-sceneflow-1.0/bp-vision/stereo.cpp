/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "filter.h"
#include "imconv.h"
#include "stereo.h"

// dt of 1d function
static void dt(float f[BP_VALUES]) {
  for (int q = 1; q < BP_VALUES; q++) {
    float prev = f[q-1] + 1.0F;
    if (prev < f[q])
      f[q] = prev;
  }
  for (int q = BP_VALUES-2; q >= 0; q--) {
    float prev = f[q+1] + 1.0F;
    if (prev < f[q])
      f[q] = prev;
  }
}
  
// compute message
void msg(float s1[BP_VALUES], float s2[BP_VALUES], 
	 float s3[BP_VALUES], float s4[BP_VALUES],
	 float dst[BP_VALUES]) {
  float val;

  // aggregate and find min
  float minimum = BP_INF;
  for (int value = 0; value < BP_VALUES; value++) {
    dst[value] = s1[value] + s2[value] + s3[value] + s4[value];
    if (dst[value] < minimum)
      minimum = dst[value];
  }

  // dt
  dt(dst);

  // truncate 
  minimum += BP_DISC_K;
  for (int value = 0; value < BP_VALUES; value++)
    if (minimum < dst[value])
      dst[value] = minimum;

  // normalize
  val = 0;
  for (int value = 0; value < BP_VALUES; value++) 
    val += dst[value];

  val /= BP_VALUES;
  for (int value = 0; value < BP_VALUES; value++) 
    dst[value] -= val;
}

// computation of data costs
image<float[BP_VALUES]> *comp_data(image<uchar> *img1, image<uchar> *img2) {
  int width = img1->width();
  int height = img1->height();
  image<float[BP_VALUES]> *data = new image<float[BP_VALUES]>(width, height);

  image<float> *sm1, *sm2;
  if (BP_SIGMA >= 0.1) {
    sm1 = smooth(img1, BP_SIGMA);
    sm2 = smooth(img2, BP_SIGMA);
  } else {
    sm1 = imageUCHARtoFLOAT(img1);
    sm2 = imageUCHARtoFLOAT(img2);
  } 

#if 0 // original version
  for (int y = 0; y < height; y++) {
    for (int x = BP_VALUES-1; x < width; x++) {
      for (int value = 0; value < BP_VALUES; value++) {
	float val = abs(imRef(sm1, x, y)-imRef(sm2, x-value, y));	
	imRef(data, x, y)[value] = BP_LAMBDA * std::min(val, BP_DATA_K);
      }
    }
  }
#else // fix by FD
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int value;
      for (value = 0; value < std::min(BP_VALUES,x); value++) {
        float val = abs(imRef(sm1, x, y)-imRef(sm2, x-value, y));
	imRef(data, x, y)[value] = BP_LAMBDA * std::min(val, BP_DATA_K);
      }
      while(value < BP_VALUES) {
	imRef(data, x, y)[value] = BP_LAMBDA * BP_DATA_K;
        value++;
      }
    }
  }
#endif

  delete sm1;
  delete sm2;
  return data;
}

// generate output from current messages
image<uchar> *output(image<float[BP_VALUES]> *u, image<float[BP_VALUES]> *d, 
		     image<float[BP_VALUES]> *l, image<float[BP_VALUES]> *r, 
		     image<float[BP_VALUES]> *data) {
  int width = data->width();
  int height = data->height();
  image<uchar> *out = new image<uchar>(width, height);

  for (int y = 1; y < height-1; y++) {
    for (int x = 1; x < width-1; x++) {
      // keep track of best value for current pixel
      int best = 0;
      float best_val = BP_INF;
      for (int value = 0; value < BP_VALUES; value++) {
	float val = 
	  imRef(u, x, y+1)[value] +
	  imRef(d, x, y-1)[value] +
	  imRef(l, x+1, y)[value] +
	  imRef(r, x-1, y)[value] +
	  imRef(data, x, y)[value];
	if (val < best_val) {
	  best_val = val;
	  best = value;
	}
      }
      imRef(out, x, y) = (uchar)(best * BP_SCALE);
    }
  }

  return out;
}

// belief propagation using checkerboard update scheme
void bp_cb(image<float[BP_VALUES]> *u, image<float[BP_VALUES]> *d,
	   image<float[BP_VALUES]> *l, image<float[BP_VALUES]> *r,
	   image<float[BP_VALUES]> *data,
	   int iter) {
  int width = data->width();  
  int height = data->height();

  (void)iter;
  for (int t = 0; t < BP_ITER; t++) {
    std::cout << "iter " << t << "\n";

    for (int y = 1; y < height-1; y++) {
      for (int x = ((y+t) % 2) + 1; x < width-1; x+=2) {

	msg(imRef(u, x, y+1),imRef(l, x+1, y),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(u, x, y));

	msg(imRef(d, x, y-1),imRef(l, x+1, y),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(d, x, y));

	msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(r, x-1, y),
	    imRef(data, x, y), imRef(r, x, y));

	msg(imRef(u, x, y+1),imRef(d, x, y-1),imRef(l, x+1, y),
	    imRef(data, x, y), imRef(l, x, y));

      }
    }
  }
}

// multiscale belief propagation for image restoration
image<uchar> *stereo_ms(image<uchar> *img1, image<uchar> *img2) {
  image<float[BP_VALUES]> *u[BP_LEVELS];
  image<float[BP_VALUES]> *d[BP_LEVELS];
  image<float[BP_VALUES]> *l[BP_LEVELS];
  image<float[BP_VALUES]> *r[BP_LEVELS];
  image<float[BP_VALUES]> *data[BP_LEVELS];

  // data costs
  data[0] = comp_data(img1, img2);

  // data pyramid
  for (int i = 1; i < BP_LEVELS; i++) {
    int old_width = data[i-1]->width();
    int old_height = data[i-1]->height();
    int new_width = (int)ceil(old_width/2.0);
    int new_height = (int)ceil(old_height/2.0);

    assert(new_width >= 1);
    assert(new_height >= 1);

    data[i] = new image<float[BP_VALUES]>(new_width, new_height);
    for (int y = 0; y < old_height; y++) {
      for (int x = 0; x < old_width; x++) {
	for (int value = 0; value < BP_VALUES; value++) {
	  imRef(data[i], x/2, y/2)[value] += imRef(data[i-1], x, y)[value];
	}
      }
    }
  }

  // run bp from coarse to fine
  for (int i = BP_LEVELS-1; i >= 0; i--) {
    int width = data[i]->width();
    int height = data[i]->height();

    // allocate & init memory for messages
    if (i == BP_LEVELS-1) {
      // in the coarsest level messages are initialized to zero
      u[i] = new image<float[BP_VALUES]>(width, height);
      d[i] = new image<float[BP_VALUES]>(width, height);
      l[i] = new image<float[BP_VALUES]>(width, height);
      r[i] = new image<float[BP_VALUES]>(width, height);
    } else {
      // initialize messages from values of previous level
      u[i] = new image<float[BP_VALUES]>(width, height, false);
      d[i] = new image<float[BP_VALUES]>(width, height, false);
      l[i] = new image<float[BP_VALUES]>(width, height, false);
      r[i] = new image<float[BP_VALUES]>(width, height, false);

      for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	  for (int value = 0; value < BP_VALUES; value++) {
	    imRef(u[i], x, y)[value] = imRef(u[i+1], x/2, y/2)[value];
	    imRef(d[i], x, y)[value] = imRef(d[i+1], x/2, y/2)[value];
	    imRef(l[i], x, y)[value] = imRef(l[i+1], x/2, y/2)[value];
	    imRef(r[i], x, y)[value] = imRef(r[i+1], x/2, y/2)[value];
	  }
	}
      }      
      // delete old messages and data
      delete u[i+1];
      delete d[i+1];
      delete l[i+1];
      delete r[i+1];
      delete data[i+1];
    } 

    // BP
    bp_cb(u[i], d[i], l[i], r[i], data[i], BP_ITER);    
  }

  image<uchar> *out = output(u[0], d[0], l[0], r[0], data[0]);

  delete u[0];
  delete d[0];
  delete l[0];
  delete r[0];
  delete data[0];

  return out;
}

int main(int argc, char **argv) {
  image<uchar> *img1, *img2, *out/*, *edges*/;

  if (argc != 4) {
    std::cerr << "usage: " << argv[0] << " left(pgm) right(pgm) out(pgm)\n";
    exit(1);
  }

  // load input
  img1 = loadPGM(argv[1]);
  img2 = loadPGM(argv[2]);

  // compute disparities
  out = stereo_ms(img1, img2);

  // save output
  savePGM(out, argv[3]);
  
  delete img1;
  delete img2;
  delete out;
  return 0;
}

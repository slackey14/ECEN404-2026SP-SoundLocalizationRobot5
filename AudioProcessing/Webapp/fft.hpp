/* * Free Fast Fourier Transform (FFT)
 * Copyright (c) 2021 Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 * * Modified to include single-precision (float) versions.
 */

#pragma once

#include <vector>
#include <complex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Fft {
	
	/*---- Double-precision functions (original) ----*/

	// Computes the discrete Fourier transform (DFT) of the given complex vector.
	void transform(std::vector<std::complex<double>> &vec);
	
	// Computes the inverse discrete Fourier transform (IDFT) of the given complex vector.
	void inverseTransform(std::vector<std::complex<double>> &vec);
	
	// Computes the DFT of the given complex vector, whose length must be a power of 2.
	void transformRadix2(std::vector<std::complex<double>> &vec);


	/*---- Single-precision functions (new) ----*/
	
	// Computes the discrete Fourier transform (DFT) of the given complex vector.
	void transform(std::vector<std::complex<float>> &vec);

	// Computes the inverse discrete Fourier transform (IDFT) of the given complex vector.
	void inverseTransform(std::vector<std::complex<float>> &vec);

	// Computes the DFT of the given complex vector, whose length must be a power of 2.
	void transformRadix2(std::vector<std::complex<float>> &vec);

}

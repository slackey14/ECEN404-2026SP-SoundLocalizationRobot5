/* * Free Fast Fourier Transform (FFT)
 * Copyright (c) 2021 Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 * * Modified to use templates for float and double precision.
 */

#include "fft.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Define M_PI if it's not already defined by cmath
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using std::complex;
using std::size_t;
using std::vector;

/* =================================================================================
 * TEMPLATED IMPLEMENTATIONS
 * These functions are the core logic and can work with any floating-point type (T).
 * =================================================================================
 */

namespace { // Use an anonymous namespace for internal linkage

template <typename T>
void transformRadix2_tmpl(vector<complex<T>> &vec) {
    // Length variables
    size_t n = vec.size();
    int levels = 0;
    for (size_t temp = n; temp > 1U; temp >>= 1)
        levels++;
    if (static_cast<size_t>(1U) << levels != n)
        throw std::domain_error("Length is not a power of 2");
    
    // Trigonometric tables
    vector<complex<T>> expTable(n / 2);
    for (size_t i = 0; i < n / 2; i++)
        expTable[i] = std::exp(complex<T>(0, -2 * M_PI * i / n));
    
    // Bit-reversed addressing permutation
    for (size_t i = 0; i < n; i++) {
        size_t j = 0;
        for (size_t temp = i, k = 0; k < levels; k++, temp >>= 1)
            j = (j << 1) | (temp & 1U);
        if (j > i)
            std::swap(vec[i], vec[j]);
    }
    
    // Cooley-Tukey decimation-in-time radix-2 FFT
    for (size_t size = 2; size <= n; size *= 2) {
        size_t halfsize = size / 2;
        size_t tablestep = n / size;
        for (size_t i = 0; i < n; i += size) {
            for (size_t j = 0; j < halfsize; j++) {
                size_t k = j * tablestep;
                complex<T> temp = vec[i + j + halfsize] * expTable[k];
                vec[i + j + halfsize] = vec[i + j] - temp;
                vec[i + j] += temp;
            }
        }
    }
}

template <typename T>
void transform_tmpl(vector<complex<T>> &vec) {
    size_t n = vec.size();
    if (n == 0)
        return;
    else if ((n & (n - 1)) == 0)  // Is power of 2
        transformRadix2_tmpl<T>(vec);
    else
        throw std::domain_error("FFT size must be a power of 2 for this implementation.");
}

template <typename T>
void inverseTransform_tmpl(vector<complex<T>> &vec) {
    std::for_each(vec.begin(), vec.end(), [](complex<T> &c){ c = std::conj(c); });
    transform_tmpl<T>(vec);
    std::for_each(vec.begin(), vec.end(), [](complex<T> &c){ c = std::conj(c); });
}

} // end anonymous namespace


/* =================================================================================
 * PUBLIC-FACING FUNCTIONS (Explicit Instantiations)
 * These connect the public declarations in fft.hpp to the templated implementations.
 * =================================================================================
 */

// Double-precision versions
void Fft::transform(vector<complex<double>> &vec) { transform_tmpl<double>(vec); }
void Fft::inverseTransform(vector<complex<double>> &vec) { inverseTransform_tmpl<double>(vec); }
void Fft::transformRadix2(vector<complex<double>> &vec) { transformRadix2_tmpl<double>(vec); }

// Single-precision versions
void Fft::transform(vector<complex<float>> &vec) { transform_tmpl<float>(vec); }
void Fft::inverseTransform(vector<complex<float>> &vec) { inverseTransform_tmpl<float>(vec); }
void Fft::transformRadix2(vector<complex<float>> &vec) { transformRadix2_tmpl<float>(vec); }


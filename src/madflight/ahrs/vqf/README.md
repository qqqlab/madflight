# VQF-C: A Lightweight Implementation of VQF for Embedded Devices

**VQF-C** is a C language implementation of the full [VQF](https://github.com/dlaidig/vqf), with credit to the original project's author for their pioneering work. This project aims to bring the full capabilities of VQF to embedded devices such as Cortex-M4F and RISC-V MCUs, with minimal overhead.

## Key Highlights
- **Full Version VQF Functionality**: The implementation retains the full feature set of the original [VQF full version](https://github.com/dlaidig/vqf/blob/main/vqf/cpp/vqf.cpp).
- **Easy Integration**: The code is structured to be easily integrated into existing C environments for embedded MCUs/chips.

## Usage
To use VQF-C in your embedded application, include the headers and utilize the provided functions.

```c
#include "vqf.h"

int main() {
    // Initialize VQF-C and perform operations
    vqf_real_t gyr[3];
    vqf_real_t acc[3];
    vqf_real_t mag[3];

    vqf_real_t quat6D[4];
    vqf_real_t quat9D[4];

    vqf_real_t gyrTs = 0.000250;
    vqf_real_t accTs = 0.001;
    vqf_real_t magTs = 5.0;
    initVqf(gyrTs, accTs, magTs);

    while (1) {
        // Update VQF-C with new sensor data
        if (newGyrData) updateGyr(gyr);
        if (newAccData) updateAcc(acc);
        if (newMagData) updateMag(mag);

        if (newGyrData || newAccData) {
            quat6D = getQuat6D();
        }

        if (newMagData) {
            quat9D = getQuat9D();
        }
    }

    return 0;
}
```
## TODO

- Integrate CMSIS-DSP library functions for mathematics and filtering to replace current implementations, aiming to achieve better runtime performance.

## License
VQF-C is open source and available under the [MIT License](https://opensource.org/license/mit). This means that you can use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the software. The full terms of the license are detailed in the [LICENSE](/LICENSE) file.

For a quick overview, here are some key points about the [MIT License](https://opensource.org/license/mit):

- **Free to use**: You can use VQF-C in your projects without any cost.
- **Permissive**: You can modify the source code and distribute your modified versions.
- **No Warranty**: The software is provided "as is" without any warranties.
- **Requiring preservation of copyright and license notices**: You must include the copyright notice and a pointer to where the MIT License can be found in any substantial portions of the software.

For the full legal text, see the [LICENSE](/LICENSE) file.

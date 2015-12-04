# FeatureBased

Feature-based 3D pose estimation on planar target

### Algorithm flow

1. feature matching & RANSAC: get the inlier correspondences between marker and camera image

2. homography estimation: estimate homography with correspondences

3. dual inverse composition refinement: refinement the homography with DIC algorithm propose by Bartoli in 2008

4. homography decomposition: estimate the final pose

### Related Publication

[1] D. G. Lowe. Distinctive image features from scale-invariant keypoints. IJCV, 60(2), 2004.

[2] G. Yu and J.-M. Morel. Asift: A new framework for fully afﬁne invariant image comparison. Image Processing On Line, 2011.

[3] M. A. Fischler and R. C. Bolles. Random sample consensus: a paradigm for model ﬁtting with applications to image analysis and automated cartography. Communications of the ACM, 24(6):381–395, 1981.

[4] Y. Zheng, Y. Kuang, S. Sugimoto, K. Astrom, and M. Okutomi. Revisiting the PnP Problem: A Fast, General and Optimal Solution. In ICCV, 2013.

[5] Harker, M., & O’Leary, P. (2005). Computation of homographies. In British computer vision conference (BMVC).

[6] Hartley, R., & Zisserman, A. (2004). Multiple view geometry in computer vision. Cambridge: Cambridge University Press.

[7] Bartoli, Adrien. "Groupwise geometric and photometric direct image registration." Pattern Analysis and Machine Intelligence, IEEE Transactions on30.12 (2008): 2098-2108.

[8] Malis, Ezio, and Manuel Vargas. "Deeper understanding of the homography decomposition for vision-based control." (2007): 90.

[9] Collins, Toby, and Adrien Bartoli. "Infinitesimal plane-based pose estimation." International Journal of Computer Vision 109.3 (2014): 252-286.

### Running

Run singleTest.m to perform a example pose estimation

### Version

v1: 20151204 by HYTseng
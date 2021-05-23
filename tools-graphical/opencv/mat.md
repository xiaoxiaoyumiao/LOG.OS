# Mat

```text
#include <opencv2/core/mat.hpp>
```

## Basics

```cpp
Mat::size()
// Returns a matrix size.

+--------+----+----+----+----+------+------+------+------+
|        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
+--------+----+----+----+----+------+------+------+------+
| CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
| CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
| CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
| CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
| CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
| CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
| CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
+--------+----+----+----+----+------+------+------+------+

// type conversion
void cv::Mat::convertTo	(	OutputArray 	m,
    int 	rtype,
    double 	alpha = 1,
    double 	beta = 0 
    )		const
    
// initialize from a C array (The content will not be copied)
double x[100][100];

cv::Mat A(100, 100, CV_64F, x);

// slicing (cropping)
cv::Mat image(imagesource); 
// Setup a rectangle to define your region of interest
cv::Rect myROI(10, 10, 100, 100);
// Crop the full image to that image contained by the rectangle myROI
// Note that this doesn't copy the data
cv::Mat croppedImage = image(myROI);

// copying data (used for padding)
cv::Mat image_padded(3, pad_size, CV_32FC1, cv::Scalar(0));
cv::Rect roi( ... );
small_image.copyTo(image_padded(roi));
```

## Mat Operations

```cpp
#include <opencv2/imgproc.hpp>


void cv::resize	(	InputArray 	src,
    OutputArray 	dst,
    Size 	dsize,
    double 	fx = 0,
    double 	fy = 0,
    int 	interpolation = INTER_LINEAR 
    )	
    
#include <opencv2/core.hpp>

void cv::dct	(	InputArray 	src,
    OutputArray 	dst,
    int 	flags = 0 
    )
    

void cv::transpose	(	InputArray 	src,
    OutputArray 	dst 
    )		


double cv::PSNR	(	InputArray 	src1,
    InputArray 	src2,
    double 	R = 255. 
    )		

void cv::split	(	const Mat & 	src,
    Mat * 	mvbegin 
    )	  
```

## Reference

\[1\] [https://stackoverflow.com/questions/44453088/how-to-convert-c-array-to-opencv-mat](https://stackoverflow.com/questions/44453088/how-to-convert-c-array-to-opencv-mat)

\[2\] [https://stackoverflow.com/questions/8267191/how-to-crop-a-cvmat-in-opencv](https://stackoverflow.com/questions/8267191/how-to-crop-a-cvmat-in-opencv)

\[3\] [https://stackoverflow.com/questions/30470020/opencv-zero-padding-of-mat](https://stackoverflow.com/questions/30470020/opencv-zero-padding-of-mat)

\[4\] [https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv](https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv)






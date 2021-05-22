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
```


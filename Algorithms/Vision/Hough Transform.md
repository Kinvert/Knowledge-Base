# Hough Transform

The **Hough Transform** is a classic feature extraction technique in image analysis, computer vision, and digital image processing. It is primarily used to detect simple geometric shapes (like lines, circles, and ellipses) within images, even if they are partially obscured or noisy. It works by transforming points from the image space into a parameter space and then identifying accumulations that correspond to particular shapes.

---

## Core Idea
- A shape in an image corresponds to a set of points satisfying a particular equation (e.g., a line equation `y = mx + b`).
- Instead of detecting shapes in image space, the Hough Transform maps image points into **parameter space**.
- Each point in the image votes for possible parameters; the most voted parameters correspond to shapes present in the image.

---

## Types
1. **Standard Hough Transform (SHT)**  
   Detects lines using the parametric form `ρ = x*cosθ + y*sinθ`.
   
2. **Probabilistic Hough Transform (PHT)**  
   A more efficient, randomized version used for large images. Operates on a subset of edge points.

3. **Circular Hough Transform (CHT)**  
   Specialized for detecting circles by parameterizing them as `(x - a)² + (y - b)² = r²`.

4. **Generalized Hough Transform (GHT)**  
   Extends the method to arbitrary shapes, not just lines/circles.

---

## One-liner Usage Examples
- Detect edges first: `edges = cv2.Canny(image, 50, 150)`
- Detect lines: `lines = cv2.HoughLines(edges, 1, np.pi/180, 200)`
- Detect circles: `circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 20)`

---

## Advantages
- Robust to noise and partial occlusion.  
- Works even when parts of the shape are missing.  
- Can be extended to different shapes with parameterized equations.

## Disadvantages
- Computationally expensive, especially in high-dimensional parameter spaces.  
- Sensitive to parameter tuning (thresholds, accumulator resolution).  
- Requires preprocessing like edge detection for good results.

---

## Comparisons

| Method                     | Use Case                           | Pros                           | Cons                                |
|-----------------------------|------------------------------------|--------------------------------|-------------------------------------|
| **Hough Transform (SHT)**   | Line detection                     | Simple, robust to noise        | Slow on large images                |
| **Probabilistic Hough (PHT)** | Faster line detection              | Efficient, works on large imgs | May miss some lines                 |
| **Circular Hough (CHT)**    | Circle detection                   | Robust to noisy circular edges | Requires radius tuning              |
| **Generalized Hough (GHT)** | Arbitrary shapes                   | Flexible                       | Computationally very expensive      |
| **Contour-based Methods**   | General shape detection            | Fast, works with OpenCV tools  | Sensitive to noise/occlusion        |
| **Template Matching**       | Specific known patterns            | Simple to implement            | Fails with rotation/scale changes   |
| **Deep Learning (CNNs)**    | Complex object detection           | Extremely powerful             | Needs large data & training compute |

---

## Examples

```vbnet
Input: Edge-detected image (e.g., Canny edges)
Initialize accumulator A[θ, ρ] to zeros

for each edge pixel (x, y) in the image:
    for θ from 0 to 180 degrees:
        ρ = x*cos(θ) + y*sin(θ)
        A[θ, ρ] += 1

Find local maxima in A → these correspond to detected lines
Return list of (ρ, θ) lines
```

```python
import cv2
import numpy as np

# Load image and convert to grayscale
img = cv2.imread("image.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Edge detection
edges = cv2.Canny(gray, 50, 150, apertureSize=3)

# Hough Transform
lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

# Draw lines on image
for line in lines:
    rho, theta = line[0]
    a, b = np.cos(theta), np.sin(theta)
    x0, y0 = a * rho, b * rho
    x1, y1 = int(x0 + 1000*(-b)), int(y0 + 1000*(a))
    x2, y2 = int(x0 - 1000*(-b)), int(y0 - 1000*(a))
    cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)

cv2.imshow("Hough Lines", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

int main() {
    // Load image and convert to grayscale
    Mat img = imread("image.jpg");
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // Edge detection
    Mat edges;
    Canny(gray, edges, 50, 150, 3);

    // Hough Transform
    std::vector<Vec2f> lines;
    HoughLines(edges, lines, 1, CV_PI/180, 200);

    // Draw lines on image
    for(size_t i=0; i<lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
        line(img, pt1, pt2, Scalar(0,0,255), 2, LINE_AA);
    }

    imshow("Hough Lines", img);
    waitKey(0);
    return 0;
}
```

---

## Related [[links]]
- [[Computer Vision]]
- [[Edge Detection]]
- [[OpenCV]]
- [[Feature Extraction]]
- [[Fourier Transform]]

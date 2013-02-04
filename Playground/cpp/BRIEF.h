#ifndef _BRIEF_H_
#define _BRIEF_H_

#include <vector>
#include <bitset>
#include <random>

using namespace std;

namespace ICSL { 
namespace Quadrotor {
class BRIEF
{
public:

  typedef bitset<256> briefbit;

  /// Type of pairs
  enum Type
  {
    RANDOM, // random pairs (Calonder's original version)
    RANDOM_CLOSE, // random but close pairs (used in GalvezIROS11)
  };

public:

  /**
   * Creates the BRIEF a priori data for descriptors of nbits length
   * @param nbits descriptor length in bits
   * @param patch_size 
   * @param type type of pairs to generate
   */
  BRIEF(int nbits = 256, int patch_size = 48, Type type = RANDOM_CLOSE):
    m_bit_length(nbits), m_patch_size(patch_size), m_type(type)
{
  //assert(patch_size > 1);
  //assert(nbits > 0);

  generateTestPoints();
}
  virtual ~BRIEF();

  /**
   * Returns the descriptor length in bits
   * @return descriptor length in bits
   */
  inline int getDescriptorLengthInBits() const
  {
    return m_bit_length;
  }

  /**
   * Returns the type of classifier
   */
  inline Type getType() const
  {
    return m_type;
  }

  /**
   * Returns the size of the patch
   */
  inline int getPatchSize() const
  {
    return m_patch_size;
  }

  /**
   * Returns the BRIEF descriptors of the given keypoints in the given image
   * @param image
   * @param points
   * @param descriptors 
   * @param treat_image (default: true) if true, the image is converted to 
   *   grayscale if needed and smoothed. If not, it is assumed the image has
   *   been treated by the user
   * @note this function is similar to BRIEF::compute
   */
  inline void operator() (const cv::Mat &image,
    const std::vector<cv::KeyPoint> &points,
    std::vector<briefbit> &descriptors,
    bool treat_image = true) const
  {
    compute(image, points, descriptors, treat_image);
  }

  /**
   * Returns the BRIEF descriptors of the given keypoints in the given image
   * @param image
   * @param points
   * @param descriptors 
   * @param treat_image (default: true) if true, the image is converted to 
   *   grayscale if needed and smoothed. If not, it is assumed the image has
   *   been treated by the user
   * @note this function is similar to BRIEF::operator()
   */
void compute(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &points,
    vector<briefbit> &descriptors,
    bool treat_image) const
{
  const float sigma = 2.f;
  const cv::Size ksize(9, 9);

  cv::Mat im;
  if(treat_image)
  {
    cv::Mat aux;
    if(image.depth() == 3)
    {
      cv::cvtColor(image, aux, CV_RGB2GRAY);
    }
    else
    {
      aux = image;
    }

    cv::GaussianBlur(aux, im, ksize, sigma, sigma);

  }
  else
  {
    im = image;
  }

  assert(im.type() == CV_8UC1);
  assert(im.isContinuous());

  // use im now
  const int W = im.cols;
  const int H = im.rows;

  descriptors.resize(points.size());
  std::vector<briefbit>::iterator dit;

  std::vector<cv::KeyPoint>::const_iterator kit;

  int x1, y1, x2, y2;

  dit = descriptors.begin();
  for(kit = points.begin(); kit != points.end(); ++kit, ++dit)
  {
    //dit->resize(m_bit_length);
    dit->reset();
    for(unsigned int i = 0; i < m_x1.size(); ++i)
    {
      x1 = (int)(kit->pt.x + m_x1[i]);
      y1 = (int)(kit->pt.y + m_y1[i]);
      x2 = (int)(kit->pt.x + m_x2[i]);
      y2 = (int)(kit->pt.y + m_y2[i]);

      if(x1 >= 0 && x1 < W && y1 >= 0 && y1 < H
        && x2 >= 0 && x2 < W && y2 >= 0 && y2 < H)
      {
        if( im.ptr<unsigned char>(y1)[x1] < im.ptr<unsigned char>(y2)[x2] )
        {
          dit->set(i);
        }
      } // if (x,y)_1 and (x,y)_2 are in the image

    } // for each (x,y)
  } // for each keypoint
}

  /**
   * Exports the test pattern
   * @param x1 x1 coordinates of pairs
   * @param y1 y1 coordinates of pairs
   * @param x2 x2 coordinates of pairs
   * @param y2 y2 coordinates of pairs
   */
  inline void exportPairs(std::vector<int> &x1, std::vector<int> &y1,
    std::vector<int> &x2, std::vector<int> &y2) const
  {
    x1 = m_x1;
    y1 = m_y1;
    x2 = m_x2;
    y2 = m_y2;
  }

  /**
   * Sets the test pattern
   * @param x1 x1 coordinates of pairs
   * @param y1 y1 coordinates of pairs
   * @param x2 x2 coordinates of pairs
   * @param y2 y2 coordinates of pairs
   */
  inline void importPairs(const std::vector<int> &x1,
    const std::vector<int> &y1, const std::vector<int> &x2,
    const std::vector<int> &y2)
  {
    m_x1 = x1;
    m_y1 = y1;
    m_x2 = x2;
    m_y2 = y2;
    m_bit_length = x1.size();
  }
 
  /**
   * Returns the Hamming distance between two descriptors
   * @param a first descriptor vector
   * @param b second descriptor vector
   * @return hamming distance
   */
  inline static int distance(const briefbit &a, const briefbit &b)
  {
    return (a^b).count();
  }

protected:

  /**
   * Generates random points in the patch coordinates, according to 
   * m_patch_size and m_bit_length
   */
void generateTestPoints()
{
  m_x1.resize(m_bit_length);
  m_y1.resize(m_bit_length);
  m_x2.resize(m_bit_length);
  m_y2.resize(m_bit_length);

  const float g_mean = 0.f;
  const float g_sigma = 0.2f * (float)m_patch_size;
  const float c_sigma = 0.08f * (float)m_patch_size;

  float sigma2;
  if(m_type == RANDOM)
    sigma2 = g_sigma;
  else
    sigma2 = c_sigma;

  const int max_v = m_patch_size / 2;

  mt19937 generator;  // a core engine class
  normal_distribution<float> normal(g_mean, g_sigma);

  for(int i = 0; i < m_bit_length; ++i)
  {
    int x1, y1, x2, y2;

    do
    {
	x1 = normal(generator);
    } while( x1 > max_v || x1 < -max_v);

    do
    {
	y1 = normal(generator);
    } while( y1 > max_v || y1 < -max_v);

    float meanx, meany;
    if(m_type == RANDOM)
      meanx = meany = g_mean;
    else
    {
      meanx = x1;
      meany = y1;
    }

    do
    {
	x2 = normal(generator);
    } while( x2 > max_v || x2 < -max_v);

    do
    {
	y2 = normal(generator);
    } while( y2 > max_v || y2 < -max_v);

    m_x1[i] = x1;
    m_y1[i] = y1;
    m_x2[i] = x2;
    m_y2[i] = y2;
  }

}
 
protected:

  /// Descriptor length in bits
  int m_bit_length;

  /// Patch size
  int m_patch_size;
 
  /// Type of pairs
  Type m_type;

  /// Coordinates of test points relative to the center of the patch
  std::vector<int> m_x1, m_x2;
  std::vector<int> m_y1, m_y2;

};
}// end namespace bracket
}
#endif

# Set output path
SET(TESTING_OUTPUT_PATH "${Strategy_BINARY_DIR}/Testing/Temporary")
MAKE_DIRECTORY(${TESTING_OUTPUT_PATH})

# Configure the Executables
ADD_EXECUTABLE(
  StrategyTests
  StrategyTest01.cxx
  StrategyTest02.cxx
  StrategyTest03.cxx
  StrategyTest04.cxx
)
TARGET_LINK_LIBRARIES(StrategyTests ${ITK_LIBRARIES})

# Configure the Tests
ADD_TEST(
  UnaryPixelMath-F2-Abs
  StrategyTests
  TestUnaryPixelMathF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-UnaryPixelMath-F2-Abs.mha
  0 # Strategy
)
ADD_TEST(
  UnaryPixelMath-UC2-InvertIntensity
  StrategyTests
  TestUnaryPixelMathUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-UnaryPixelMath-UC2-InvertIntensity.mha
  8 # Strategy
)
ADD_TEST(
  UnaryPixelMath-F2-InvertIntensity
  StrategyTests
  TestUnaryPixelMathF2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-UnaryPixelMath-F2-InvertIntensity.mha
  8 # Strategy
  255.0 # InvertIntensityMaximum
)
ADD_TEST(
  UnaryPixelMath-F2-Sigmoid
  StrategyTests
  TestUnaryPixelMathF2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-UnaryPixelMath-F2-Sigmoid.mha
  12 # Strategy
  0 # InvertIntensityMaximum  (not used)
  50 0 # SigmoidAlpha SigmoidBeta
)
ADD_TEST(
  UnaryPixelMath-F2-Square
  StrategyTests
  TestUnaryPixelMathF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-UnaryPixelMath-F2-Square.mha
  15 # Strategy
)

ADD_TEST(
  BinaryPixelMath-SS2-Add
  StrategyTests
  TestBinaryPixelMathSS2
  ${DATA_PATH}/cthead1.png
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-BinaryPixelMath-SS2-Add.mha
  0 # Strategy
)
ADD_TEST(
  BinaryPixelMath-UC2-Mask
  StrategyTests
  TestBinaryPixelMathUC2
  ${DATA_PATH}/cthead1.png
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/cthead1-BinaryPixelMath-UC2-Mask.mha
  9 # Strategy
)
ADD_TEST(
  BinaryPixelMath-F2-WeightedAdd
  StrategyTests
  TestBinaryPixelMathF2
  ${DATA_PATH}/cthead1.png
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/cthead1-BinaryPixelMath-F2-WeightedAdd.mha
  4 # Strategy
  0.3 # WeightedAddAlpha
)

ADD_TEST(
  DistanceMap-F2-Danielsson
  StrategyTests
  TestDistanceMapF2
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/Spots-DistanceMap-F2-Danielsson.mha
  0 # Strategy
)
ADD_TEST(
  DistanceMap-F2-SignedDanielsson
  StrategyTests
  TestDistanceMapF2
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/Spots-DistanceMap-F2-SignedDanielsson.mha
  1 # Strategy
)
ADD_TEST(
  DistanceMap-F2-SignedMaurer
  StrategyTests
  TestDistanceMapF2
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/Spots-DistanceMap-F2-SignedMaurer.mha
  2 # Strategy
)

ADD_TEST(
  Gradient-UC2-FiniteDifference
  StrategyTests
  TestGradientUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Gradient-UC2-FiniteDifference.mha
  0 # Strategy
)
ADD_TEST(
  Gradient-UC2-RecursiveGaussian
  StrategyTests
  TestGradientUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Gradient-UC2-RecursiveGaussian.mha
  1 # Strategy
)
ADD_TEST(
  Gradient-F2-FiniteDifference
  StrategyTests
  TestGradientF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Gradient-F2-FiniteDifference.mha
  0 # Strategy
)
ADD_TEST(
  Gradient-F2-RecursiveGaussian
  StrategyTests
  TestGradientF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Gradient-F2-RecursiveGaussian.mha
  1 # Strategy
)

ADD_TEST(
  GradientMagnitude-UC2-FiniteDifference
  StrategyTests
  TestGradientMagnitudeUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-UC2-FiniteDifference.mha
  0 # Strategy
)
ADD_TEST(
  GradientMagnitude-UC2-RecursiveGaussian
  StrategyTests
  TestGradientMagnitudeUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-UC2-RecursiveGaussian.mha
  1 # Strategy
  2.0 # Sigma
)
ADD_TEST(
  GradientMagnitude-F2-RecursiveGaussian
  StrategyTests
  TestGradientMagnitudeF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-F2-RecursiveGaussian.mha
  1 # Strategy
  2.0 # Sigma
)
ADD_TEST(
  GradientMagnitude-UC2-Morphological-1
  StrategyTests
  TestGradientMagnitudeUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-UC2-Morphological-1.mha
  2 # Strategy
  0 # Sigma
  1 # KernelRadius
)
ADD_TEST(
  GradientMagnitude-UC2-Morphological-2
  StrategyTests
  TestGradientMagnitudeUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-UC2-Morphological-2.mha
  2 # Strategy
  0 # Sigma
  2 # KernelRadius
)
ADD_TEST(
  GradientMagnitude-F2-Laplacian
  StrategyTests
  TestGradientMagnitudeF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-F2-Laplacian.mha
  3 # Strategy
)
ADD_TEST(
  GradientMagnitude-F2-LaplacianRecursiveGaussian
  StrategyTests
  TestGradientMagnitudeF2
  ${DATA_PATH}/cthead1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-GradientMagnitude-F2-LaplacianRecursiveGaussian.mha
  4 # Strategy
  2.0 # Sigma
)

ADD_TEST(
  Threshold-UC2-Binary
  StrategyTests
  TestThresholdUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Binary.mha
  0 # Strategy
  0 # Outside
  255 # Inside
  100 # Lower
  200 # Upper
)
ADD_TEST(
  Threshold-UC2-Single
  StrategyTests
  TestThresholdUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Single.mha
  1 # Strategy
  0 # Outside
  0 # Inside (ignored for Single)
  100 # Lower
  200 # Upper
)
ADD_TEST(
  Threshold-UC2-Otsu
  StrategyTests
  TestThresholdUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Otsu.mha
  3 # Strategy
  0 # Outside
  255 # Inside
)
ADD_TEST(
  Threshold-UC2-KappaSigma
  StrategyTests
  TestThresholdUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-KappaSigma.mha
  3 # Strategy
  0 # Outside
  255 # Inside
)

ADD_TEST(
  Projection-UC3UC2-Maximum-0
  StrategyTests
  TestProjectionUC3UC2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-UC3UC2-Maximum-0.mha
  2 # Strategy
  0 # ProjectionDimension
)
ADD_TEST(
  Projection-UC3UC2-Maximum-1
  StrategyTests
  TestProjectionUC3UC2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-UC3UC2-Maximum-1.mha
  2 # Strategy
  1 # ProjectionDimension
)
ADD_TEST(
  Projection-UC3UC2-Maximum-2
  StrategyTests
  TestProjectionUC3UC2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-UC3UC2-Maximum-2.mha
  2 # Strategy
  2 # ProjectionDimension
)
ADD_TEST(
  Projection-F3F2-Sum-0
  StrategyTests
  TestProjectionF3F2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-F3F2-Sum-0.mha
  4 # Strategy
  0 # ProjectionDimension
)
ADD_TEST(
  Projection-F3F2-Sum-1
  StrategyTests
  TestProjectionF3F2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-F3F2-Sum-1.mha
  4 # Strategy
  1 # ProjectionDimension
)
ADD_TEST(
  Projection-F3F2-Sum-2
  StrategyTests
  TestProjectionF3F2
  ${DATA_PATH}/engine.mha
  ${TESTING_OUTPUT_PATH}/engine-Projection-F3F2-Sum-2.mha
  4 # Strategy
  2 # ProjectionDimension
)

ADD_TEST(
  Morphology-UC2-Dilate
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-Dilate.mha
  0 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-Erode
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-Erode.mha
  1 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-Close
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-Close.mha
  2 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-Open
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-Open.mha
  3 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-BinaryDilate
  StrategyTests
  TestMorphologyUC2
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Binary.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-BinaryDilate.mha
  4 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-BinaryErode
  StrategyTests
  TestMorphologyUC2
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Binary.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-BinaryErode.mha
  5 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-BinaryClose
  StrategyTests
  TestMorphologyUC2
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Binary.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-BinaryClose.mha
  6 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-BinaryOpen
  StrategyTests
  TestMorphologyUC2
  ${TESTING_OUTPUT_PATH}/cthead1-Threshold-UC2-Binary.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-BinaryOpen.mha
  7 # Strategy
  3 # KernelLength
)
ADD_TEST(
  Morphology-UC2-BlackTopHat
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-BlackTopHat.mha
  10 # Strategy
  5 # KernelLength
)
ADD_TEST(
  Morphology-UC2-WhiteTopHat
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-WhiteTopHat.mha
  11 # Strategy
  5 # KernelLength
)
ADD_TEST(
  Morphology-F2-HConcave
  StrategyTests
  TestMorphologyF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Morphology-F2-HConcave.mha
  20 # Strategy
)
ADD_TEST(
  Morphology-F2-HConvex
  StrategyTests
  TestMorphologyF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Morphology-F2-HConvex.mha
  21 # Strategy
)
ADD_TEST(
  Morphology-F2-HMaxima
  StrategyTests
  TestMorphologyF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Morphology-F2-HMaxima.mha
  22 # Strategy
)
ADD_TEST(
  Morphology-F2-HMinima
  StrategyTests
  TestMorphologyF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Morphology-F2-HMinima.mha
  23 # Strategy
)
ADD_TEST(
  Morphology-UC2-RegionalMaxima
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-RegionalMaxima.mha
  24 # Strategy
)
ADD_TEST(
  Morphology-UC2-RegionalMinima
  StrategyTests
  TestMorphologyUC2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Morphology-UC2-RegionalMinima.mha
  25 # Strategy
)

ADD_TEST(
  Smooth-UC2-Bilateral
  StrategyTests
  TestSmoothUC2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Smooth-UC2-Bilateral.mha
  0 # Strategy
  10.0 # Sigma
)
ADD_TEST(
  Smooth-F2-CurvatureFlow
  StrategyTests
  TestSmoothF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Smooth-F2-CurvatureFlow.mha
  1 # Strategy
)
ADD_TEST(
  Smooth-F2-DiscreteGaussian
  StrategyTests
  TestSmoothF2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Smooth-F2-DiscreteGaussian.mha
  2 # Strategy
  3.0 # Sigma
)
ADD_TEST(
  Smooth-UC2-RecursiveGaussian
  StrategyTests
  TestSmoothUC2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Smooth-UC2-RecursiveGaussian.mha
  3 # Strategy
  3.0 # Sigma
)
ADD_TEST(
  Smooth-UC2-Median
  StrategyTests
  TestSmoothUC2
  ${DATA_PATH}/closerec1.jpg
  ${TESTING_OUTPUT_PATH}/closerec1-Smooth-UC2-Median.mha
  4 # Strategy
  5.0 # Sigma
)

ADD_TEST(
  Registration-F2-T-RSGD-MS-L-1
  StrategyTests
  TestRegistrationF2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Moving-F2-T-RSGD-MS-L-1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Output-F2-T-RSGD-MS-L-1.mha
  00 # Transform
  09 # Optimizer
  10 # Metric
  01 # Interpolator
  20 20 # InitialParameters
)
ADD_TEST(
  Registration-F2-T-RSGD-KS-NN-1
  StrategyTests
  TestRegistrationF2
  ${DATA_PATH}/Spots.png
  ${TESTING_OUTPUT_PATH}/Spots-Registration-Moving-F2-T-RSGD-KS-NN-1.mha
  ${TESTING_OUTPUT_PATH}/Spots-Registration-Output-F2-T-RSGD-KS-NN-1.mha
  00 # Transform
  09 # Optimizer
  04 # Metric
  00 # Interpolator
  -25 -25 # InitialParameters
)
ADD_TEST(
  Registration-F2-E2D-RSGD-NCC-BS-1
  StrategyTests
  TestRegistrationF2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Moving-F2-E2D-RSGD-NCC-BS-1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Output-F2-E2D-RSGD-NCC-BS-1.mha
  02 # Transform
  09 # Optimizer
  13 # Metric
  02 # Interpolator
  0.5 0.0 0.0 # InitialParameters
)
ADD_TEST(
  Registration-F2-SM2-RSGD-NCC-BS-1
  StrategyTests
  TestRegistrationF2
  ${DATA_PATH}/cthead1.png
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Moving-F2-SM2-RSGD-NCC-BS-1.mha
  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Output-F2-SM2-RSGD-NCC-BS-1.mha
  08 # Transform
  09 # Optimizer
  13 # Metric
  02 # Interpolator
  1.4 0.0 0.0 0.0 # InitialParameters
  0.5 1.0 1.0 1.0 # Scales
)
#ADD_TEST(
#  Registration-F2-AF-RSGD-MMI-BS-1
#  StrategyTests
#  TestRegistrationF2
#  ${DATA_PATH}/cthead1.png
#  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Moving-F2-AF-RSGD-MMI-BS-1.mha
#  ${TESTING_OUTPUT_PATH}/cthead1-Registration-Output-F2-AF-RSGD-MMI-BS-1.mha
#  10 # Transform
#  09 # Optimizer
#  07 # Metric
#  02 # Interpolator
#  1.4 0.0 0.0 0.0 # InitialParameters
#  0.5 1.0 1.0 1.0 # Scales
#)
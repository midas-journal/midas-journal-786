ADD_EXECUTABLE(
  Strategy
  itkStrategyFactory.h
  itkStrategyFactory.txx
  itkSimpleBinaryPixelMathImageFilter.h
  itkSimpleDistanceMapImageFilter.h
  itkSimpleGradientImageFilter.h
  itkSimpleGradientMagnitudeImageFilter.h
  itkSimpleImageToImageMetric.h
  itkSimpleInterpolateImageFunction.h
  itkSimpleMorphologyImageFilter.h
  itkSimpleOptimizer.h
  itkSimpleProjectionImageFilter.h
  itkSimpleSmoothImageFilter.h
  itkSimpleThresholdImageFilter.h
  itkSimpleTransform.h
  itkSimpleUnaryPixelMathImageFilter.h
  main.cxx
)
TARGET_LINK_LIBRARIES(Strategy ${ITK_LIBRARIES})
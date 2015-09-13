
#idndef ATTRACTION_BASSIN_BUILDER_H
#define ATTRACTION_BASSIN_BUILDER_H

namespace TeachRepeat
{

  class AttractionBassinBuilder
  {
  public:
    AttractionBassinBuilder(const AnchorPoint& reference, const AnchorPoint& reading);
    setReading(const AnchorPoint newReading);
  private:
    AnchorPoint mReference;
    AnchorPoint mReading;
  };

} // Namespace TeachRepeat.

#endif

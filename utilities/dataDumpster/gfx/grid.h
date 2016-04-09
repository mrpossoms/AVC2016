#pragma mark

#include "renderer.h"

namespace gfx {

class Grid : Drawable {
public:
	Grid(int rows, int cols);
	~Grid();

	void draw(Renderer* renderer);
private:
	int rows, cols;
};

}

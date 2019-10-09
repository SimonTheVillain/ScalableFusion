#ifndef FILE_VIEW_H
#define FILE_VIEW_H

/**
 * This is an ambitious interface to handle keyboard input within the opengl context
 * e.g. when multiple layers of rendering should be able to react to mouse and keyboard inputs
 */
namespace gfx {

class ViewController {
public:

	virtual bool isCursorResponder() {return true;}
	virtual bool isOnlyCursorResponder() {return true;}//says if the cursor should get captured
	virtual bool isKeyboardResponder() {return true;}
	virtual bool isOnlyKeyboardResponder() {return true;}
	//now stuff to capture all this keyboard events
};

} // namespace gfx

#endif // FILE_VIEW_H

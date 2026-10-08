#ifndef CSUBSYSTEM_H
#define CSUBSYSTEM_H

#include <string>

// labelled part of the robot with its own internal state
// e.g a drive motor - derive from this class 
class CSubsystem {
	public:
    	CSubsystem(const std::string& label);

    	// virtual destructor so derived subsystems clean up correctly 
    	virtual ~CSubsystem();

    	// returns the subsystem's label.
    	const std::string& GetName() const;

    	// update according to subsystem 
    	virtual void Update() = 0;

    	// Reports subsystem's label and current state
    	virtual void Report() const = 0;

  	private:
    	std::string Name;      // label
};

#endif

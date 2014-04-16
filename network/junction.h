#ifndef JUNCTION_H
#define JUNCTION_H

enum class TerminalType { NONE, EAST, WEST };

class Junction {
public:
    int             id;
    bool            is_terminal;
    TerminalType    terminal_type;
    
    Junction(const int id, const bool is_terminal, const TerminalType terminal_type) : id(id), is_terminal(is_terminal), terminal_type(terminal_type) {}
};

#endif
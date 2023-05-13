#include <cstdio>
#include <cctype>
#include <string>
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <optional>
#include <unordered_map>
#include <map>

/*
When possible put in ParseOne
use Variant with visit lambda with if-else chain
do much better error reporting for parsing errors
*/

const bool debug_state = false;

namespace Console
{
    void Print(const std::string& str, State& state)
    {
        std::cout << str;
        size_t newline_at = str.find_last_of('\n');
        if(newline_at == std::string::npos) {
            state.column += str.size();
        } else {
            state.column += str.size() - newline_at;
        }
    }

    void Tab(int tabstop, State& state)
    {
        int needed = tabstop - state.column % tabstop;
        std::cout << std::string(needed, ' ');
    }
}


enum TokenType
{
    STRING_IDENTIFIER,
    NUMBER_IDENTIFIER,
    DOUBLE,
    INTEGER,
    STRING,
    REMARK,  		 // Remark (comment) starting with REM keyword
    ABS,
    ATN,
    COS,
    EXP,
    INT,
    LOG,
    RND,
    SGN,
    SIN,
    SQR,
    TAN,
    LEFT,
    RIGHT,
    MID,
    LEN,
    STR,
    TAB,
    VAL,
    CHR,
    NOT_EQUAL,
    LESS_THAN,
    GREATER_THAN,
    LESS_THAN_EQUAL,
    GREATER_THAN_EQUAL,
    OPEN_PAREN,
    CLOSE_PAREN,
    EQUAL,
    PLUS,
    MINUS,
    MULTIPLY,
    DIVIDE,
    AND,
    OR,
    NOT,
    POWER,
    COMMA,
    COLON,
    SEMICOLON,
    WAIT,
    DEF,
    FN,
    DIM,
    LET,
    IF,
    THEN,
    ELSE,
    FOR,
    TO,
    STEP,
    NEXT,
    GOTO,
    GOSUB,
    RETURN,
    PRINT,
    INPUT,
    END,
    WIDTH,
    CLEAR,
    RUN,
    STOP,
    ON,
    READ,
    ORDER,
    DATA,
    TOKENTYPE_END,
};


std::unordered_map<TokenType, int> operator_precedence = {
    { ABS, -1000 },
    { ATN, -1000 },
    { COS, -1000 },
    { EXP, -1000 },
    { INT, -1000 },
    { LOG, -1000 },
    { RND, -1000 },
    { SGN, -1000 },
    { SIN, -1000 },
    { SQR, -1000 },
    { TAN, -1000 },
    { LEFT, -1000 },
    { RIGHT, -1000 },
    { MID, -1000 },
    { LEN, -1000 },
    { STR, -1000 },
    { TAB, -1000 },
    { VAL, -1000 },
    { CHR, -1000 },
    { POWER, 6 }, 
    { MULTIPLY, 5 }, 
    { DIVIDE, 5 }, 
    { PLUS, 4 }, 
    { MINUS, 4 }, 
    { LESS_THAN, 3 }, 
    { GREATER_THAN, 3 }, 
    { LESS_THAN_EQUAL, 3 }, 
    { GREATER_THAN_EQUAL, 3 }, 
    { EQUAL, 2 }, 
    { NOT_EQUAL, 2 }, 
    { AND, 2 }, 
    { OR, 2 }, 
};

std::set<TokenType> function_tokens = {
    ABS,
    ATN,
    COS,
    EXP,
    INT,
    LOG,
    RND,
    SGN,
    SIN,
    SQR,
    TAN,
    LEFT,
    RIGHT,
    MID,
    LEN,
    STR,
    TAB,
    VAL,
    CHR,
};

std::set<TokenType> operator_tokens = {
    POWER, 
    MULTIPLY, 
    DIVIDE, 
    PLUS, 
    MINUS, 
    LESS_THAN, 
    GREATER_THAN, 
    LESS_THAN_EQUAL, 
    GREATER_THAN_EQUAL, 
    EQUAL, 
    NOT_EQUAL, 
    AND, 
    OR, 
    NOT, 
};

std::set<TokenType> binary_operators = {
    POWER, 
    MULTIPLY, 
    DIVIDE, 
    PLUS, 
    MINUS, 
    LESS_THAN, 
    GREATER_THAN, 
    LESS_THAN_EQUAL, 
    GREATER_THAN_EQUAL, 
    EQUAL, 
    NOT_EQUAL, 
    AND, 
    OR, 
};

std::set<TokenType> unary_operators = {
    PLUS, 
    MINUS, 
    NOT,
};

std::set<TokenType> commands = {
    WAIT,
    DEF,
    FN,
    DIM,
    LET,
    IF,
    THEN,
    ELSE,
    FOR,
    TO,
    STEP,
    NEXT,
    GOTO,
    GOSUB,
    RETURN,
    PRINT,
    INPUT,
    END,
    WIDTH,
    CLEAR,
    RUN,
    STOP,
    ON,
    READ,
    ORDER,
    DATA,
};


std::unordered_map<std::string, TokenType> StringToToken =
{
    {"<>", NOT_EQUAL},
    {"<=", LESS_THAN_EQUAL},
    {">=", GREATER_THAN_EQUAL},
    {">", GREATER_THAN_EQUAL},
    {"<", LESS_THAN},
    {"(", OPEN_PAREN},
    {")", CLOSE_PAREN},
    {"=", EQUAL},
    {"+", PLUS},
    {"-", MINUS},
    {"*", MULTIPLY},
    {"/", DIVIDE},
    {"^", POWER},
    {",", COMMA},
    {":", COLON},
    {";", SEMICOLON},
    {"DIM", DIM},
    {"LET", LET},
    {"IF", IF},
    {"THEN", THEN},
    {"ELSE", ELSE},
    {"FOR", FOR},
    {"TO", TO},
    {"STEP", STEP},
    {"NEXT", NEXT},
    {"GOTO", GOTO},
    {"GOSUB", GOSUB},
    {"RETURN", RETURN},
    {"PRINT", PRINT},
    {"INPUT", INPUT},
    {"END", END},
    {"ABS", ABS},
    {"ATN", ATN},
    {"COS", COS},
    {"EXP", EXP},
    {"INT", INT},
    {"LOG", LOG},
    {"RND", RND},
    {"SGN", SGN},
    {"SIN", SIN},
    {"SQR", SQR},
    {"TAN", TAN},
    {"AND", AND},
    {"OR", OR},
    {"NOT", NOT},
    {"LEFT$", LEFT},
    {"RIGHT$", RIGHT},
    {"MID$", MID},
    {"STR$", STR},
    {"LEN", LEN},
    {"TAB", TAB},
    {"VAL", VAL},
    {"WAIT", WAIT},
    {"DEF", DEF},
    {"FN", FN},
    {"CHR$", CHR},
    {"WIDTH", WIDTH},
    {"CLEAR", CLEAR},
    {"ON", ON},
    {"READ", READ},
    {"ORDER", ORDER},
    {"DATA", DATA},
    {"RUN", RUN},
    {"STOP", STOP},
};

std::unordered_map<TokenType, const char *> TokenTypeToStringMap =
{
    {EQUAL, "="},
    {NOT_EQUAL, "<>"},
    {LESS_THAN_EQUAL, "<="},
    {GREATER_THAN_EQUAL, ">="},
    {LESS_THAN, "<"},
    {GREATER_THAN, ">"},
    {OPEN_PAREN, "("},
    {CLOSE_PAREN, ")"},
    {PLUS, "+"},
    {MINUS, "-"},
    {MULTIPLY, "*"},
    {DIVIDE, "/"},
    {POWER, "^"},
    {COMMA, ","},
    {COLON, ":"},
    {SEMICOLON, ";"},
    {DIM, "DIM"},
    {LET, "LET"},
    {IF, "IF"},
    {THEN, "THEN"},
    {ELSE, "ELSE"},
    {FOR, "FOR"},
    {TO, "TO"},
    {STEP, "STEP"},
    {NEXT, "NEXT"},
    {GOTO, "GOTO"},
    {GOSUB, "GOSUB"},
    {RETURN, "RETURN"},
    {PRINT, "PRINT"},
    {INPUT, "INPUT"},
    {END, "END"},
    {ABS, "ABS"},
    {ATN, "ATN"},
    {COS, "COS"},
    {EXP, "EXP"},
    {INT, "INT"},
    {LOG, "LOG"},
    {RND, "RND"},
    {SGN, "SGN"},
    {SIN, "SIN"},
    {SQR, "SQR"},
    {TAN, "TAN"},
    {AND, "AND"},
    {OR, "OR"},
    {NOT, "NOT"},
    {LEFT, "LEFT$"},
    {RIGHT, "RIGHT$"},
    {MID, "MID$"},
    {STR, "STR$"},
    {LEN, "LEN"},
    {TAB, "TAB"},
    {VAL, "VAL"},
    {WAIT, "WAIT"},
    {DEF, "DEF"},
    {FN, "FN"},
    {CHR, "CHR$"},
    {WIDTH, "WIDTH"},
    {CLEAR, "CLEAR"},
    {ON, "ON"},
    {READ, "READ"},
    {ORDER, "ORDER"},
    {DATA, "DATA"},
    {RUN, "RUN"},
    {STOP, "STOP"},
};

struct TokenizeError
{
    enum Type
    {
        SYNTAX
    } type;
    int position;
    TokenizeError(Type type, int position) :
        type(type),
        position(position)
    {}
};

std::string str_toupper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::toupper(c); }
                  );
    return s;
}

struct VariableReference
{
    std::string name;
    std::vector<int> indices;
    VariableReference(const std::string& name, const std::vector<int>& indices) :
        name(name),
        indices(indices)
    {}
};

typedef std::variant<std::string, double, VariableReference> Value;
double to_basic_bool(bool b) { return b ? -1 : 0; }

struct Token
{
    public:
        TokenType type;
        Value value{0.0};

        Token(TokenType type) : type(type) {}
        Token(TokenType type, const std::string& value) : type(type), value(value) {}
        Token(int32_t value) : type(INTEGER), value(static_cast<double>(value)) {}
        Token(double value) : type(DOUBLE), value(value) {}
        operator TokenType() {return type; }
        operator Value() {return value; }
};

std::string str(const Value& v) { return std::get<std::string>(v); }
double num(const Value& v) { return std::get<double>(v); }
int32_t igr(const Value& v) { return static_cast<int32_t>(std::get<double>(v)); }
VariableReference vref(const Value& v) { return std::get<VariableReference>(v); }
bool is_vref(const Value& v) { return std::holds_alternative<VariableReference>(v); }
bool is_str(const Value& v) { return std::holds_alternative<std::string>(v); }
bool is_num(const Value& v) { return std::holds_alternative<double>(v); }


typedef std::vector<Token> TokenList;
typedef TokenList::const_iterator TokenIterator;

TokenList Tokenize(const std::string& line)
{
    TokenList tokens;
    std::string pending;
    size_t pending_started = 0;

    auto add_pending = [&](int index, char c) {
        if(pending.empty()) {
            pending_started = index;
        }
        pending += c;
    };

    auto flush_pending = [&]() {
        while (!pending.empty())
        {
            std::size_t ipos{}, dpos{};
            bool found_integer = false, found_float = false;
            int32_t i;
            double d;
            try {
                i = std::stoi(pending, &ipos);
                found_integer = true;
            } catch(std::invalid_argument const& ex) {
                found_integer = false;
            }

            try {
                d = std::stod(pending, &dpos);
                found_float = true;
            } catch(std::invalid_argument const& ex) {
                found_float = false;
            }

            if(found_integer && (!found_float || dpos <= ipos)) {
                tokens.push_back(Token(i));
                pending = pending.substr(ipos);
                pending_started += ipos;
            } else if(found_float) {
                tokens.push_back(Token(d));
                pending = pending.substr(dpos);
                pending_started += dpos;
            } else {
                for(int i = 0; i < pending.size() - 1; i++) {
                    char c = pending[i];
                    if(!isalnum(c) && c != '_') {
                        throw TokenizeError(TokenizeError::SYNTAX, pending_started + i);
                    }
                }
                char c = pending[pending.size() - 1];
                if(!isalnum(c) && c != '_' && c != '$') {
                    throw TokenizeError(TokenizeError::SYNTAX, pending_started + i);
                }
                if(pending[pending.size() - 1] == '$') {
                    tokens.push_back(Token(STRING_IDENTIFIER, str_toupper(pending)));
                } else {
                    tokens.push_back(Token(NUMBER_IDENTIFIER, str_toupper(pending)));
                }
                pending.clear();
                pending_started = std::string::npos;
            }
        }
    };

    for (size_t index = 0; index < line.size();) {
        if(str_toupper(line.substr(index, 3)) == "REM") {
            flush_pending();
            tokens.push_back(Token(REMARK, line.substr(index + 3)));
            break;
        }

        char c = line[index];

        if (isspace(c))
        {
            index++;
            continue;
        }

        if(c == '"') {
            flush_pending();
            index++;
            std::string str;
            while(index < line.size() && line[index] != '"') {
                str += line[index];
                index++;
            }
            if(index < line.size()) {
                index++;
            }
            tokens.push_back(Token(STRING, str));
            continue;
        }

        auto result = [&]() -> std::optional<std::pair<size_t, TokenType>>{
            // TODO Could probably make a custom compare that would fit std::map
            for(const auto& [word, result]: StringToToken) {
                if(str_toupper(line.substr(index, word.size())) == word.c_str()) {
                    return std::make_pair(word.size(), result);
                }
            }
            return std::nullopt;
        }();

        if (result) {
            flush_pending();
            auto size = result.value().first;
            auto token = result.value().second;
            tokens.push_back(Token(token));
            index += size;
        } else {
            add_pending(index, c);
            index++;
        }
    }

    flush_pending();

    return tokens;
}

void PrintTokenized(const TokenList& tokens, int emphasize = -1)
{
    printf("%zd tokens: ", tokens.size());
    int which = 0;
    for(const auto& t: tokens) {
        if(which == emphasize) {
            printf(" ==>");
        }
        switch(t.type) {
            case STRING_IDENTIFIER:
            case NUMBER_IDENTIFIER:
            {
                auto v = t.value;
                printf("%s ", std::get<std::string>(v).c_str());
                break;
            }
            case DOUBLE: {
                auto v = t.value;
                printf("%f ", std::get<double>(v));
                break;
            }
            case INTEGER: {
                auto v = t.value;
                printf("%d ", igr(v));
                break;
            }
            case REMARK: {
                auto v = t.value;
                printf("REM%s ", std::get<std::string>(v).c_str());
                break;
            }
            case STRING: {
                auto v = t.value;
                printf("\"%s\" ", std::get<std::string>(v).c_str());
                break;
            }
            default: {
                printf("%s ", TokenTypeToStringMap[t.type]);
                break;
            }
        }
        if(which++ == emphasize) {
            printf("<== ");
        }
    }
    printf("\n");
}

std::tuple<Value, Value> pop2(std::vector<Value>& operands)
{
    Value right = operands.back(); operands.pop_back();
    Value left = operands.back(); operands.pop_back();
    return {left, right};
}

template <typename Q>
auto pop(Q& queue)
{
    auto back = queue.back();
    queue.pop_back();
    return back;
}

void dump_state(const std::vector<std::string>& operators, const std::vector<Value>& operands)
{
    printf("[");
    for(auto op: operators) { printf("\"%s\" ", op.c_str()); }
    printf("] (");
    for(auto op: operands) {
        if(is_num(op)) {
            printf("%f ", num(op));
        } else {
            printf("\"%s\" ", str(op).c_str());
        }
    }
    printf(")");
}

void dump_operators(const std::vector<std::string>& operators)
{
    printf("operators: ");
    for(auto op: operators) { printf("\"%s\" ", op.c_str()); }
    printf("\n");
}

void dump_operands(const std::vector<Value>& operands)
{
    printf("operands: ");
    for(auto op: operands) {
        if(is_num(op)) {
            printf("%f ", num(op));
        } else {
            printf("\"%s\" ", str(op).c_str());
        }
    }
    printf("\n");
}

Value evaluate(TokenType op, std::vector<Value>& operands)
{
    Value right = pop(operands);
    if(op == CLOSE_PAREN) {
        return right;
    } else if(op == OPEN_PAREN) {
        return right;
    } else if(op == POWER) {
        Value left = num(pop(operands));
        return pow(num(left), num(right));
    } else if(op == MULTIPLY) {
        Value left = pop(operands);
        return num(left) * num(right);
    } else if(op == DIVIDE) {
        Value left = pop(operands);
        return num(left) / num(right);
    } else if(op == PLUS) {
        Value left = pop(operands);
        return num(left) + num(right);
    } else if(op == MINUS) {
        Value left = pop(operands);
        return num(left) - num(right);
    } else if(op == TAB) {
	// TODO this is wrong, should add spaces from current location
	// to the specified next tab stop
        return std::string(static_cast<int>(num(right)), ' ');
    } else if(op == SIN) {
        return sin(num(right));
    } else if(op == COS) {
        return cos(num(right));
    } else if(op == TAN) {
        return tan(num(right));
    } else if(op == SGN) {
        return num(right) < 0.0 ? -1.0 : (num(right) > 0.0 ? 1.0 : 0.0);
    } else if(op == INT) {
        return trunc(num(right));
    } else if(op == RND) {
        return drand48();
    } else if(op == EXP) {
        return exp(num(right));
    } else if(op == LOG) {
        return log(num(right));
    } else {
        printf("internal error\n");
        abort();
    }
}

std::optional<TokenType> is_operator(TokenIterator op, TokenIterator end)
{
    if((op < end) && (operator_tokens.count(op->type) > 0)) {
        return op->type;
    }
    return std::nullopt;
}

bool is_binary_operator(TokenType op)
{
    return binary_operators.count(op) > 0;
}

bool is_unary_operator(TokenType op)
{
    return unary_operators.count(op) > 0;
}

bool is_function(TokenType&word)
{
    return function_tokens.count(word) > 0;
}

bool is_command(TokenType&command)
{
    return commands.count(command) > 0;
}

bool is_higher_precedence(TokenType op1, TokenType op2)
{
    bool both_binary = is_binary_operator(op1) && is_binary_operator(op2);
    bool is_higher = both_binary && (operator_precedence.at(op1) > operator_precedence.at(op2));
    return is_higher;
}

std::string to_str(const Value& v)
{
    if(is_vref(v)) {
        auto ref = vref(v);
        std::string s = ref.name;
        if(ref.indices.size() > 0) {
            s = s + "(" + std::to_string(ref.indices[0]);
            for(auto it = ref.indices.begin() + 1; it < ref.indices.end(); it++) {
                s = s + ", " + std::to_string(*it);
            }
            s = s + ")";
        }
        return s;
    } else if(is_num(v)) {
        return std::to_string(num(v));
    } else {
        return str(v);
    }
}


struct State
{
    std::unordered_map<std::string, Value> variables;
    std::map<int, TokenList> program;
    int current_line{-1};
    int goto_line{-1};
    bool direct{true};
    int column{0};
};

struct ParseError
{
    enum {
        UNEXPECTED_END,
        UNEXPECTED,
        EXPECTED_TOKEN,
        EXPECTED_TERM,
    } type;
    TokenType expected_token_type{TOKENTYPE_END};
    TokenList tokens;
    int token{-1};
    std::string expected_term;
    ParseError(TokenList tokens, TokenType expected_token, int token) :
        type(EXPECTED_TOKEN),
        expected_token_type(expected_token),
        tokens(tokens),
        token(token)
    {}
    ParseError(TokenList tokens, const std::string& expected_term, int token) :
        type(EXPECTED_TERM),
        tokens(tokens),
        expected_term(expected_term)
    {}
    ParseError(TokenList tokens, int token) :
        type(UNEXPECTED),
        tokens(tokens),
        token(token)
    {}
    ParseError(TokenList tokens) :
        type(UNEXPECTED_END),
        tokens(tokens)
    {}
};

std::optional<Token> ParseOptional(const TokenList& tokens, TokenIterator& cur, TokenIterator& end, State& state, const std::set<TokenType>& expect)
{
    if((cur < end) && (expect.count(cur->type) > 0)) {
        return *cur++;
    }
    return {};
}

std::optional<Token> ParseAny(const TokenList& tokens, TokenIterator& cur, TokenIterator& end, State& state, const std::set<TokenType>& expect)
{
    if(cur >= tokens.end()) { return {}; }
    if(expect.count(cur->type) > 0) {
        return (cur++)->type;
    }
    return {};
}

std::optional<Token> ParseOne(const TokenList& tokens, TokenIterator& cur, TokenIterator& end, State& state, TokenType expect)
{
    if(cur >= tokens.end()) { return {}; }
    if(cur->type == expect) {
        return (cur++)->type;
    }
    return {};
}

bool IsOneOf(TokenType type, const std::set<TokenType>& expect)
{
    return expect.count(type) > 0;
}

// identifier ::= NUMBER_IDENTIFIER | STRING_IDENTIFIER // returns optional std::string
std::optional<Value> ParseIdentifier(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    if(cur_ >= end) { return {}; }
    if(IsOneOf(cur_->type, {NUMBER_IDENTIFIER, STRING_IDENTIFIER})) {
        return cur_++->value;
    }
    return {};
}

// number ::= DOUBLE | INTEGER // returns optional Value
std::optional<Value> ParseNumber(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    if(cur_ >= end) { return {}; }
    if(IsOneOf(cur_->type, {DOUBLE, INTEGER})) {
        return cur_++->value;
    }
    return {};
}

// unary-op ::= (PLUS | MINUS | NOT) // returns TokenType
std::optional<TokenType> ParseUnaryOp(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    if(cur_ >= end) { return {}; }
    if(IsOneOf(cur_->type, {PLUS, MINUS, NOT})) {
        return cur_++->type;
    }
    return {};
}

// integer-list ::= INTEGER {COMMA INTEGER} // returns std::vector<int>
std::optional<std::vector<int>> ParseIntegerList(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    std::vector<int> integers;
    if(cur_ >= end) { return {}; }
    if(cur_->type != INTEGER) {
        return {};
    }
    auto cur = cur_;
    integers.push_back(igr(cur++->value));
    while((cur < end) && (cur->type == COMMA)) {
        cur++;
        if(cur->type != INTEGER) {
            return {};
        }
        integers.push_back(igr(cur++->value));
    }
    cur_ = cur;
    return integers;
}

// number-identifier-list ::= NUMBER_IDENTIFIER {COMMA NUMBER_IDENTIFIER} // returns std::vector<Token>
std::optional<std::vector<Value>> ParseNumberIdentifierList(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    std::vector<Value> identifiers;
    if(cur_ >= end) { return {}; }
    if(cur_->type != NUMBER_IDENTIFIER) {
        return {};
    }
    auto cur = cur_;
    identifiers.push_back(cur++->value);
    while((cur < end) && (cur->type == COMMA)) {
        cur++;
        if(cur->type != NUMBER_IDENTIFIER) {
            return {};
        }
        identifiers.push_back(cur++->value);
    }
    cur_ = cur;
    return identifiers;
}

// numeric-function-name ::= ABS | ATN | COS | EXP | INT | LOG | RND | SGN | SIN | SQR | TAN | TAB | CHR | STR // returns TokenType
std::optional<TokenType> ParseNumericFunctionName(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    if(cur_ >= end) { return {}; }
    if(IsOneOf(cur_->type, {ABS, ATN, COS, EXP, INT, LOG, RND, SGN, SIN, SQR, TAN, TAB, CHR, STR})) {
        return cur_++->type;
    }
    return {};
}

// integer ::= INTEGER // returns optional int32_t
std::optional<Value> ParseInteger(const TokenList& tokens, TokenIterator& cur_, TokenIterator end)
{
    if(cur_ >= end) { return {}; }
    if(cur_->type != INTEGER) {
        return {};
    }
    return cur_++->value;
}

bool ParseSingleWordStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state, TokenType expected)
{
    auto cur = cur_;

    if(cur >= end || cur_->type != expected) {
        return false;
    }
    cur++;

    // Committed from here, must emit parse error if can't match
    if(cur >= end) {
        cur_ = cur;
        return true;
    }

    if(cur->type == COLON) {
        cur++;
        cur_ = cur;
        return true;
    }

    throw ParseError(tokens, cur - tokens.begin());
}


// end-statement ::= END (COLON | end)// returns void
bool ParseEndStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, END);
    if(succeeded) {
        // TODO clear variables
    }
    return succeeded;
}

// clear-statement ::= CLEAR (COLON | end)// returns void
bool ParseClearStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, CLEAR);
    if(succeeded) {
        // TODO clear variables
    }
    return succeeded;
}

// run-statement ::= RUN (COLON | end)// returns void
bool ParseRunStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, RUN);
    if(succeeded) {
        // TODO enter run state
    }
    return succeeded;
}

// stop-statement ::= STOP (COLON | end) // returns void
bool ParseStopStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, STOP);
    if(succeeded) {
        // TODO stop run state
    }
    return succeeded;
}

// goto-statement ::= GOTO integer (COLON | end) // returns void
std::optional<int32_t> ParseGotoStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(cur_ >= end) { return {}; }

    if(cur_->type != GOTO) {
        return {};
    }

    // Committed from here, must emit parse error if can't match

    auto cur = cur_ + 1;
    if(cur >= end) {
        throw ParseError(tokens);
    }

    if(cur->type != INTEGER) {
        throw ParseError(tokens, INTEGER, cur - tokens.begin());
    }
    int goto_line = igr(cur++->value);

    if(cur >= end || ParseOne(tokens, cur, end, state, COLON)) {
        cur_ = cur;
        return state.goto_line = goto_line;
    }

    throw ParseError(tokens, cur - tokens.begin());
}

// term ::= number | STRING | variable-reference | function | paren-expression // evaluates using unary-ops, returns Value
std::optional<Value> ParseTerm(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(cur_ >= end) { return {}; }

    if(auto results = ParseNumber(tokens, cur_, end)) {
        return *results;
    }

    if(cur_->type == STRING) {
        return cur_++->value;
    }

#if 0 // TODO
    if(auto results = ParseVariableReference(tokens, cur_ ,end, state)) {
        return *results;
    }
    if(auto results = ParseFunction(tokens, cur_, end, state)) {
        return *results;
    }
    if(auto results = ParseParenExpression(tokens, cur_, end, state)) {
        return *results;
    }
#endif

    return {};
}

// unary-operation ::= {unary-op} term // evaluates using unary-ops, returns Value
std::optional<Value> ParseUnaryOperation(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(cur_ >= end) { return {}; }

    auto cur = cur_;
    std::vector<TokenType> operators;
    while(auto unary_op = ParseUnaryOp(tokens, cur, end)) {
        operators.push_back(*unary_op);
    }

    if(auto value = ParseTerm(tokens, cur, end, state)) {
        if(!is_num(*value)) {
            return {};
        }
        double v = num(*value);
        for(auto it = operators.rbegin(); it != operators.rend(); it++) {
            switch(*it) {
                case TokenType::PLUS: break;
                case TokenType::MINUS: v = -v; break;
                case TokenType::NOT: v = -1 - static_cast<int>(trunc(v)); break;
                default: break;
            }
        }
        cur_ = cur;
        return v;
    }

    return {};
}

// expression ::= paren-expression | STRING | operation | function // evaluates, returns Value
std::optional<Value> ParseExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    // XXX for bringup, remove
    // This way I can test with simple values
    if(auto results = ParseUnaryOperation(tokens, cur_, end, state)) {
        return *results;
    }

    if((cur_ < end) && (cur_->type == STRING)) {
        return cur_++->value;
    }

#if 0
    // TODO
    if(auto results = ParseParenExpression(tokens, cur_, end, state)) {
        return *results;
    }
    if(auto results = ParseOperation(tokens, cur_, end, state)) {
        return *results;
    }
    if(auto results = ParseFunction(tokens, cur_, end, state)) {
        return *results;
    }
#endif

    return {};
}

// print-statement ::= PRINT {COMMA | SEMICOLON | expression} (COLON | END) // returns void
bool ParsePrintStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(cur_ >= end) { return false; }

    if(cur_->type != PRINT) {
        return false;
    }
    // Committed from here, must emit parse error if can't match
    auto cur = cur_ + 1;

    bool lastWasConcat = false;

    while(cur < end && cur->type != COLON) {
        if(cur->type == COMMA) {
            Console::Tab(20, state);
            lastWasConcat = true;
            cur++;
        } else if(cur->type == SEMICOLON) {
            // skip
            lastWasConcat = true;
            cur++;
        } else if(auto results = ParseExpression(tokens, cur, end, state)) {
            Console::Print(to_str(*results), state);
            lastWasConcat = false;
        } else {
            throw ParseError(tokens, cur - tokens.begin());
        }
    } 

    if(!lastWasConcat) {
        Console::Print("\n", state);
    }

    if(cur->type == COLON) {
        cur++;
    }

    cur_ = cur;
    return true;
}

// statement ::= ( print-statement | let-statement | input-statement | dim-statement | if-statement | for-statement | next-statement | on-statement | goto-statement | gosub-statement | wait-statement | width-statement | order-statement | read-statement | data-statement | deffn-statement | return-statement | end-statement | clear-statement | run-statement | stop-statement ) // returns void
void ParseStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(ParseEndStatement(tokens, cur_, end, state)) {
        printf("end\n");
        return;
    } else if(ParseClearStatement(tokens, cur_, end, state)) {
        printf("clear\n");
        return;
    } else if(ParseRunStatement(tokens, cur_, end, state)) {
        printf("run\n");
        return;
    } else if(ParseStopStatement(tokens, cur_, end, state)) {
        printf("stop\n");
        return;
    } else if(ParseGotoStatement(tokens, cur_, end, state)) {
        printf("goto %d\n", state.goto_line);
        return;
    } else if(ParsePrintStatement(tokens, cur_, end, state)) {
        // print
        return;
#if 0
    // TODO
    } else if(ParseLetStatement(tokens, cur_, end, state)) {
        printf("let\n");
        return;
    } else if(ParseInputStatement(tokens, cur_, end, state)) {
        printf("input\n");
        return;
    } else if(ParseDimStatement(tokens, cur_, end, state)) {
        printf("Dim\n");
        return;
    } else if(ParseIfStatement(tokens, cur_, end, state)) {
        printf("If\n");
        return;
    } else if(ParseForStatement(tokens, cur_, end, state)) {
        printf("For\n");
        return;
    } else if(ParseNextStatement(tokens, cur_, end, state)) {
        printf("Next\n");
        return;
    } else if(ParseOnStatement(tokens, cur_, end, state)) {
        printf("On\n");
        return;
    } else if(ParseGosubStatement(tokens, cur_, end, state)) {
        printf("Gosub\n");
        return;
    } else if(ParseWaitStatement(tokens, cur_, end, state)) {
        printf("Wait\n");
        return;
    } else if(ParseWidthStatement(tokens, cur_, end, state)) {
        printf("Width\n");
        return;
    } else if(ParseOrderStatement(tokens, cur_, end, state)) {
        printf("Order\n");
        return;
    } else if(ParseReadStatement(tokens, cur_, end, state)) {
        printf("Read\n");
        return;
    } else if(ParseDataStatement(tokens, cur_, end, state)) {
        printf("Data\n");
        return;
    } else if(ParseDefStatement(tokens, cur_, end, state)) {
        printf("Def\n");
        return;
    } else if(ParseReturnStatement(tokens, cur_, end, state)) {
        printf("Return\n");
        return;
#endif
    }
    throw ParseError(tokens, cur_ - tokens.begin());
}

// statement-list ::= {statement} // returns void
void ParseStatementList(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    while(cur_ < end) {
        ParseStatement(tokens, cur_, end, state);
    }
}

// Start ::= statement-list
void Parse(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    // Either this succeeds or throws a parse error
    ParseStatementList(tokens, cur_, end, state);
}

/* 
variable-reference ::= identifier [OPEN_PAREN numeric-expression {COMMA numeric-expression} CLOSE_PAREN] // returns VariableReference
paren-expression ::= OPEN_PAREN expression CLOSE_PAREN // returns Value
numeric-expression ::= expression that is a number // returns double
integer-expression ::= numeric-expression that is an int // returns int
string-expression ::= expression resulting in STRING // returns std::string?
numeric-function ::= numeric-function-name OPEN_PAREN numeric-expression CLOSE_PAREN  // returns TokenType
len-function ::= LEN OPEN_PAREN string-expression CLOSE_PAREN
val-function ::= VAL OPEN_PAREN string-expression CLOSE_PAREN
val-function ::= LEFT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
val-function ::= RIGHT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
val-function ::= MID OPEN_PAREN string-expression COMMA numeric-expression [COMMA numeric-expression] CLOSE_PAREN
user-function ::= FN NUMBER_IDENTIFIER OPEN_PAREN numeric-expression [COMMA numeric-expression] CLOSE_PAREN
function ::= numeric-function |
             len-function |
             val-function |
             left-function |
             right-function |
             mid-function |
             user-function
             // evaluates, returns Value
operation ::= expression (POWER | MULTIPLY | DIVIDE | PLUS | MINUS | LESS_THAN | GREATER_THAN | LESS_THAN_EQUAL | GREATER_THAN_EQUAL | EQUAL | NOT_EQUAL | AND | OR) expression // evaluates in correct order, returns Value
expression-list ::= expression {COMMA expression} // returns std::vector<Value>
variable-reference-list ::= variable-reference {COMMA | variable-reference} // returns std::vector<VariableReference>
let-statement ::= [LET] variable-reference EQUAL expression (COLON | end) // returns void
input-statement ::= INPUT [STRING SEMICOLON] variable-reference-list (COLON | end) // returns void
dim-statement ::= DIM identifier OPEN_PAREN integer-list CLOSE_PAREN (COLON | end) // returns void
if-statement ::= IF expression THEN (integer | statement) [ELSE (integer | statement)] (COLON | end) // returns void
for-statement ::= FOR NUMBER_IDENTIFIER EQUAL expression TO expression [STEP expression] (COLON | end) // Only number identifiers? (COLON | end) // returns void
next-statement ::= NEXT [NUMBER_IDENTIFIER] (COLON | end) // returns void
on-statement ::= ON integer-expression (GOTO | GOSUB) integer-list (COLON | end) // returns void
gosub-statement ::= GOSUB integer (COLON | end) // returns void
wait-statement ::= WAIT numeric-expression (COLON | end) // returns void
width-statement ::= WIDTH numeric-expression (COLON | end) // returns void
order-statement ::= ORDER INTEGER (COLON | end) // returns void
read-statement ::= READ variable-reference-list (COLON | end) // returns void
data-statement ::= DATA expression-list (COLON | end) // returns void
deffn-statement ::= DEF FN NUMBER_IDENTIFIER OPEN_PAREN number-identifier-list CLOSE_PAREN numeric-expression (COLON | end) // returns void
no need to do this one: line ::= INTEGER statement-list EOL | statement-list EOL // returns void
return-statement ::= RETURN // returns void

std::optional<Token> ParseOptional(const TokenList& tokens, TokenIterator& cur, TokenIterator& end, State& state, const std::set<TokenType>& expect);
std::optional<Token> ParseAny(const TokenList& tokens, TokenIterator& cur, TokenIterator& end, State& state, const std::set<TokenType>& expect);
bool IsOneOf(TokenType type, const std::set<TokenType>& expect);


Parse...
    // If cur is a TokenIterator& with value, the value is cur->value

    // If contents not optional,
    if(cur_ >= end) { return {}; }

    // Make Local Copy
    auto cur = cur_;

    // On Success
    cur_ = cur;
    return thing-for-success;

    // On failure
    return {};

*/

#if 0
bool ParseAssignment(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;
    VariableReference reference;

    ParseOptional(tokens, cur, end, {LET});

    if(auto results = ParseAny(tokens, cur, end, state, {STRING_IDENTIFIER, NUMBER_IDENTIFIER}) {
        variable = str(results->value);
        if(!ParseOnly(tokens, cur, end, state, EQUAL) {
            throw ParseError(tokens, EQUAL, cur - tokens.begin()); }
        }
        auto value = ParseExpression(tokens, cur, end, state);
        if(!value) {
            throw ParseError(tokens, cur - tokens.begin()); }
        }
        // TODO string variables
        state.variables[str_toupper(str(*results))] = *value;
        return true;
    }
    return false;
}
#endif

void EvaluateTokens(const TokenList& tokens, State& state)
{
    TokenIterator cur = tokens.begin();
    std::vector<Value> operands;
    std::vector<std::string> operators;
    std::vector<std::string> unary_operators;

    bool next_operator_is_unary = true;

    if(cur >= tokens.end()) { throw ParseError(tokens); }
    if(cur->type == INTEGER) {
        int line_number = static_cast<int>(num(tokens.at(0).value));
        auto line = state.program[line_number];
        std::copy(tokens.begin() + 1, tokens.end(), std::back_inserter(line));
        return;
    }

    /* XXX for bringup */ PrintTokenized(tokens);

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto value = ParseTerm(tokens, cur, end, state)) {
            printf("term: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto value = ParseUnaryOperation(tokens, cur, end, state)) {
            printf("unary operation: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto identifier = ParseIdentifier(tokens, cur, end)) {
            printf("identifier \"%s\"\n", str(*identifier).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto number = ParseNumber(tokens, cur, end)) {
            printf("number %f\n", num(*number));
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto integers = ParseIntegerList(tokens, cur, end)) {
            printf("integer list (%zd) ", integers->size());
            for(auto i: *integers) {
                printf("%d, ", i);
            }
            printf("\n");
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto integer = ParseInteger(tokens, cur, end)) {
            printf("integer %d\n", igr(*integer));
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto identifiers = ParseNumberIdentifierList(tokens, cur, end)) {
            printf("identifier list (%zd) ", identifiers->size());
            for(auto v: *identifiers) {
                printf("%s, ", str(v).c_str());
            }
            printf("\n");
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto ttype = ParseUnaryOp(tokens, cur, end)) {
            printf("unary op %d %s\n", *ttype, TokenTypeToStringMap[*ttype]);
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = tokens.begin();
        TokenIterator end = tokens.end();

        if(auto ttype = ParseNumericFunctionName(tokens, cur, end)) {
            printf("numeric function name %d %s\n", *ttype, TokenTypeToStringMap[*ttype]);
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    TokenIterator end = tokens.end();
    Parse(tokens, cur, end, state);
    printf("    %zd tokens remaining \n", end - cur);

#if 0
    while(line[cur]) {
        double number = -666;

        skip_whitespace();

        if(line[cur] == ';') {
            used = 1;
        } else if(line[cur] == ',') {
            used = 1;
        } else if(line[cur] == '(') {
            operators.push_back("(");
            next_operator_is_unary = true;
            used = 1;
        } else if(line[cur] == ')') {
            bool did_an_unwind = false;
            while(!operators.empty() && operators.back() != "(") {
                if(debug_state) { printf("unwinding to \"(\" :"); dump_state(operators, operands); puts("");}
                did_an_unwind = true;
                std::string op2 = pop(operators);
                // printf("finishing lower-precedence operator %s before \")\"\n", op2.c_str());
                operands.push_back(evaluate(op2, operands));
            }
            if(operators.empty()) {
                printf("unexpected end parenthesis\n");
                abort();
            }
            if(did_an_unwind) {
                if(debug_state) { printf("after unwinding to \"(\" :"); dump_state(operators, operands); puts(""); }
            }
            operators.pop_back();
            if(!operators.empty() && is_function(operators.back())) {
                std::string func = pop(operators);
                // printf("finishing function operator %s before \")\"\n", func.c_str());
                operands.push_back(evaluate(func, operands));
                // printf("after function %s before \")\"\n", func.c_str());
            }
            next_operator_is_unary = false;
            used = 1;
        } else if(auto result = is_operator(line + cur, &used)) {
            auto op = *result;
            if(next_operator_is_unary) {
                if(is_unary_operator(op)) {
                    unary_operators.push_back(op);
                } else {
                    printf("unexpected operator in unary context: \"%s\"\n", op.c_str());
                    abort();
                }
                if(debug_state) { printf("unary operator :"); dump_state(operators, operands); puts(""); }
            } else {
                bool did_an_unwind = false;
                while(!operators.empty() && is_higher_precedence(operators.back(), op)) {
                    did_an_unwind = true;
                    if(debug_state) { printf("unwinding higher precedence :"); dump_state(operators, operands); puts(""); }
                    std::string higher = pop(operators);
                    // printf("%s is higher precedence than %s\n", higher.c_str(), op.c_str());
                    operands.push_back(evaluate(higher, operands));
                }
                if(did_an_unwind) {
                    if(debug_state) { printf("after unwinding higher precedence :"); dump_state(operators, operands); puts(""); }
                }
                operators.push_back(op);
                next_operator_is_unary = is_binary_operator(op);
            }
        } else if(sscanf(line + cur, "\"%[^\"]\"%n", word, &used) == 1) {
            operands.push_back(word);
            if(debug_state) { printf("operand :"); dump_state(operators, operands); puts(""); }
            next_operator_is_unary = false;
        } else if(sscanf(line + cur, "%lf%n", &number, &used) == 1) {
            while(!unary_operators.empty()) {
                std::string op = pop(unary_operators);
                if(op == "-") {
                    number = -number;
                } else if(op == "NOT") {
                    number = -1 - number;
                } else if(op == "+") {
                    // number = number;
                } else {
                    printf("internal error, unary operator \"%s\"\n", op.c_str());
                }
            }
            operands.push_back(number);
            if(debug_state) { printf("operand :"); dump_state(operators, operands); puts(""); }
            next_operator_is_unary = false;
        } else if(sscanf(line + cur, "%[A-Za-z]%n", word, &used) == 1) {
            std::string identifier{str_toupper(word)};
            if(operator_precedence.count(identifier) > 0) {
                operators.push_back(identifier);
                if(debug_state) { printf("operator :"); dump_state(operators, operands); puts(""); }
                next_operator_is_unary = true;
            } else if(state.variables.count(identifier) > 0) {
                operands.push_back(state.variables[identifier]);
                if(debug_state) { printf("variable :"); dump_state(operators, operands); puts(""); }
                next_operator_is_unary = false;
            } else {
                printf("unknown word \"%s\"\n", identifier.c_str());
                abort();
            }
        } else {
            printf("syntax error at \"%s\"\n", line + cur);
            abort();
        }
        assert(used != 0);
        cur += used;
    }
    while(!operators.empty()) {
        std::string op = pop(operators);
        if(debug_state) { printf("unwinding final operators :"); dump_state(operators, operands); puts(""); }
        // printf("finishing lower-precedence operator %s\n", op.c_str());
        operands.push_back(evaluate(op, operands));
    }
    if(debug_state) { printf("final state :"); dump_state(operators, operands); puts(""); }
#if 0
    if(operands.size() > 0) {
        printf("%s\n", to_str(operands.back()).c_str());
    }
    if(operands.size() > 1) {
        printf("extra ");
        dump_operands(operands);
    }
#endif

    if(command == "PRINT") {
        for(auto v: operands) {
            if(is_num(v)) {
                printf("%f ", num(v));
            } else {
                printf("%s ", str(v).c_str());
            }
        }
        printf("\n");
    } else if(command == "LET") {
        auto ref = vref(operands.at(0));
        auto value = operands.at(1);
        state.variables[ref.name] = value;
    } else if(command == "END") {
        state.direct = true;
    } else if(command == "RUN") {
        if(!state.direct) {
            printf("need to be in direct mode for RUN\n");
            abort();
        }
        state.current_line = state.program.begin()->first;
        state.direct = false;
        while(!state.direct) {
            state.goto_line = -1;
            evaluate_line(state.program.at(state.current_line).c_str(), state);
            if(state.direct) {
                break;
            }
            if(state.goto_line == -1) {
                auto next_line = state.program.find(state.current_line);
                next_line++;
                if(next_line == state.program.end()) {
                    state.direct = true;
                    break;
                }
                // printf("next line from %d yielded %d\n", state.current_line, next_line->first);
                state.current_line = next_line->first;
            } else {
                auto next_line = state.program.find(state.goto_line);
                if(next_line == state.program.end()) {
                    printf("unknown line number %d\n", state.current_line);
                    abort();
                }
                // printf("goto %d yielded %d\n", state.goto_line, next_line->first);
                state.current_line = next_line->first;
            }
        }
    } else if(command == "GOTO") {
        if(state.direct) {
            printf("need to be in run mode for GOTO\n");
            abort();
        }
        state.goto_line = static_cast<int>(trunc(num(operands[0])));
    } else {
        printf("unimplemented command \"%s\"\n", command.c_str());
    }
#endif
}

int main(int argc, char **argv)
{
    static char line[512];
    State state;
    while(fgets(line, sizeof(line), stdin) != nullptr) {
        line[strlen(line) - 1] = '\0';

        try {
            TokenList tokens = Tokenize(line);
            EvaluateTokens(tokens, state);
        } catch (const TokenizeError& e) {
            switch(e.type) {
                case TokenizeError::SYNTAX:
                    printf("syntax error at %d (\"%*s\")\n", e.position, std::min(5, (int)(strlen(line) - e.position)), line + e.position);
                    break;
            }
        } catch (const ParseError& e) {
            switch(e.type) {
                case ParseError::UNEXPECTED_END:
                    printf("unexpected end of tokens while parsing\n");
                    break;
                case ParseError::UNEXPECTED:
                    printf("unexpected token while parsing\n");
                    break;
                case ParseError::EXPECTED_TOKEN:
                    printf("expected %s token at %d while parsing\n", TokenTypeToStringMap[e.expected_token_type], e.token);
                    break;
                case ParseError::EXPECTED_TERM:
                    printf("expected term %s at %d while parsing\n", TokenTypeToStringMap[e.expected_token_type], e.token);
                    break;
            }
            PrintTokenized(e.tokens, e.token);
        }
    }
}

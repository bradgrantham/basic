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
all statements should really throw parse errors on !(COLON | end)
Use C++ exception classes and construct the string?
Unify console output so printing an error moves the column
use Variant with visit lambda with if-else chain
*/

const bool debug_state = false;
const bool debug_statements = false;


enum TokenType
{
    TEST, // XXX for bringup
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
    {"TEST", TEST},
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
    {TEST, "TEST"},
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
    std::vector<int32_t> indices;
    VariableReference(const std::string& name, const std::vector<int32_t>& indices) :
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
int32_t is_igr(double v) { return v == static_cast<int32_t>(v); }


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
            printf(" >>>");
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
            printf("<<< ");
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

struct VariableValue
{
    // A variable can have both a scalar and an array value
    // e.g. A = 5: A(0) = 6: PRINT A, A(0) yields "5 6"

    Value scalar;
    std::vector<int32_t> sizes;
    std::vector<Value> array;

    void SetArraySizes(const std::vector<int32_t>& sizes_, const Value& init)
    {
        sizes = sizes_;
        int32_t size = 1;
        for(int32_t s: sizes) {
            size *= (s + 1);
        }
        array.resize(size, init);
    }

    VariableValue(const Value& v) :
        scalar(v)
    {
    }

    VariableValue(const Value& v, const std::vector<int32_t>& sizes, const Value& init) :
        scalar(v)
    {
        SetArraySizes(sizes, init);
    }

    VariableValue(const std::vector<int32_t>& sizes, const Value& init)
    {
        SetArraySizes(sizes, init);
    }
};

typedef std::unordered_map<std::string, VariableValue> VariableMap;

struct VariableReferenceBoundsError
{
    std::string var;
    int32_t index;
    int32_t size;
    VariableReferenceBoundsError(const std::string var, int32_t index, int32_t size) :
        var(var),
        index(index),
        size(size)
    {}
};

struct VariableDimensionError
{
    std::string var;
    int used;
    int expected;
    VariableDimensionError(const std::string var, int used, int expected) :
        var(var),
        used(used),
        expected(expected)
    {}
};

struct ExecutionError
{
    enum Type {
        TYPE_MISMATCH,
        NOT_IN_RUN_STATE,
        NOT_IN_DIRECT_STATE,
        LINE_NOT_FOUND,
        VARIABLE_NOT_FOUND,
    } type;
    std::string why;
    ExecutionError(Type type) :
        type(type)
    {}
    ExecutionError(const std::string& why, Type type) :
        type(type),
        why(why)
    {}
};

Value EvaluateVariable(const VariableReference& ref, const VariableMap& variables)
{
    auto iter = variables.find(ref.name);
    if(iter == variables.end()) {
	// XXX it may turn out that referencing an unknown variable
	// should allocate that variable with default value.
        throw ExecutionError(ref.name, ExecutionError::VARIABLE_NOT_FOUND);
    }

    auto& val = iter->second;
    if(ref.indices.empty()) {
        return val.scalar;
    }

    if(val.sizes.size() != ref.indices.size()) {
        throw VariableDimensionError(ref.name, ref.indices.size(), val.sizes.size());
    }

    int32_t index = 0;
    int32_t stride = 1;
    for(size_t i = 0; i < val.sizes.size(); i++){
        if(ref.indices[i] > val.sizes[i]) {
            throw VariableReferenceBoundsError(ref.name, ref.indices[i], val.sizes[i]);
        }
        index = index + ref.indices[i] * stride;
        stride = stride * val.sizes[i];
    }
    return val.array[index];
}

void AllocateVariable(const VariableReference& ref, VariableMap& variables)
{
    if(!ref.indices.empty()) {
        std::vector<int32_t> sizes;
        for(size_t i = 0; i < ref.indices.size(); i++) {
            sizes.push_back(10);
        }
        if(ref.name[ref.name.size() - 1] == '$') {
            variables.emplace(std::make_pair(ref.name, VariableValue("", sizes, "")));
        } else {
            variables.emplace(std::make_pair(ref.name, VariableValue(0.0, sizes, 0.0)));
        }
    } else {
        if(ref.name[ref.name.size() - 1] == '$') {
            variables.emplace(std::make_pair(ref.name, VariableValue("")));
        } else {
            variables.emplace(std::make_pair(ref.name, VariableValue(0.0)));
        }
    }
}

void AssignVariable(const VariableReference& ref, const Value& value, VariableMap& variables)
{
    auto iter = variables.find(ref.name);

    if(iter == variables.end()) {
        AllocateVariable(ref, variables);
    } else {
        VariableValue& vv = variables.at(ref.name);
        if(!ref.indices.empty()) {
            if(ref.indices.size() != vv.sizes.size()) {
                throw VariableDimensionError(ref.name, ref.indices.size(), vv.sizes.size());
            }
        }
    }

    VariableValue& vv = variables.at(ref.name);

    if(!ref.indices.empty()) {
        int32_t index = 0;
        int32_t stride = 1;
        for(size_t i = 0; i < vv.sizes.size(); i++){
            if(ref.indices[i] > vv.sizes[i]) {
                throw VariableReferenceBoundsError(ref.name, ref.indices[i], vv.sizes[i]);
            }
            index = index + ref.indices[i] * stride;
            stride = stride * vv.sizes[i];
        }
        vv.array[index] = value;
    } else {
        vv.scalar = value;
    }
}


struct State
{
    VariableMap variables;
    std::map<int, TokenList> program;
    int current_line{-1};
    int goto_line{-1};
    bool direct{true};
    int column{0};
};

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


struct ParseError
{
    enum Type {
        TRAILING_TOKENS,
        UNEXPECTED_END,
        UNEXPECTED,
        EXPECTED_TOKEN,
        EXPECTED_RULE,
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
        type(EXPECTED_RULE),
        tokens(tokens),
        expected_term(expected_term)
    {}

    // UNEXPECTED or TRAILING_TOKENS
    ParseError(TokenList tokens, Type type, int token) :
        type(type),
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

// If next token is "expect", then increment pointer and return matched token.
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

std::optional<double> ParseNumericExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state);

// numeric-function ::= numeric-function-name OPEN_PAREN numeric-expression CLOSE_PAREN  // returns TokenType
std::optional<Value> ParseNumericFunction(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;

    auto function = ParseNumericFunctionName(tokens, cur, end);
    if(!function.has_value()) {
        return {};
    }

    if(!ParseOne(tokens, cur, end, state, OPEN_PAREN)) {
        return {};
    }

    auto argument = ParseNumericExpression(tokens, cur, end, state);
    if(!argument.has_value()) {
        return {};
    }

    if(!ParseOne(tokens, cur, end, state, CLOSE_PAREN)) {
        return {};
    }

    Value v;
    switch(*function) {
        case ABS:
            v = abs(*argument);
            break;
        case ATN:
            v = atan(*argument);
            break;
        case COS:
            v = cos(*argument);
            break;
        case EXP:
            v = exp(*argument);
            break;
        case INT:
            v = trunc(*argument);
            break;
        case LOG:
            v = log(*argument);
            break;
        case RND:
            v = drand48();
            break;
        case SGN:
            v = (*argument) < 0.0 ? -1.0 : ((*argument) > 0.0 ? 1.0 : 0.0);
            break;
        case SIN:
            v = sin(*argument);
            break;
        case SQR:
            v = sqrt(*argument);
            break;
        case TAN:
            v = tan(*argument);
            break;
        case TAB:
            // TODO this is wrong, should add spaces from current location
            // to the specified next tab stop
            v = std::string(static_cast<int>(*argument), ' ');
            break;
        case CHR: {
            char c = static_cast<int>(*argument);
            v = std::string(1, c);
            break;
        }
        case STR:
            if(is_igr(*argument)) {
                v = std::to_string(static_cast<int>(*argument));
            } else {
                v = std::to_string(*argument);
            }
            break;
        default:
            // notreached
            break;
    }

    cur_ = cur;
    return v;
}

// function ::= numeric-function | len-function | val-function | left-function | right-function | mid-function | user-function // evaluates, returns Value
std::optional<Value> ParseFunction(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(auto value = ParseNumericFunction(tokens, cur_, end, state)) {
        return value;
    }
#if 0
    // TODO
    if(auto value = ParseLenFunction(tokens, cur_, end, state)) {
        return value;
    }
    if(auto value = ParseValFunction(tokens, cur_, end, state)) {
        return value;
    }
    if(auto value = ParseLeftFunction(tokens, cur_, end, state)) {
        return value;
    }
    if(auto value = ParseRightFunction(tokens, cur_, end, state)) {
        return value;
    }
    if(auto value = ParseMidFunction(tokens, cur_, end, state)) {
        return value;
    }
    if(auto value = ParseUserFunction(tokens, cur_, end, state)) {
        return value;
    }
#endif
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

    throw ParseError(tokens, ParseError::UNEXPECTED, cur - tokens.begin());
}


// end-statement ::= END (COLON | end)// returns void
bool ParseEndStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, END);
    if(succeeded) {
        if(state.direct) {
            throw ExecutionError("END command", ExecutionError::NOT_IN_DIRECT_STATE);
        }
        state.direct = false;
    }
    return succeeded;
}

// clear-statement ::= CLEAR (COLON | end)// returns void
bool ParseClearStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, CLEAR);
    if(succeeded) {
        state.variables.clear();
    }
    return succeeded;
}

// run-statement ::= RUN (COLON | end)// returns void
bool ParseRunStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, RUN);
    if(succeeded) {
        if(!state.direct) {
            throw ExecutionError("RUN command", ExecutionError::NOT_IN_DIRECT_STATE);
        }
        state.current_line = state.program.begin()->first;
        state.direct = false;
    }
    return succeeded;
}

// stop-statement ::= STOP (COLON | end) // returns void
bool ParseStopStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    bool succeeded = ParseSingleWordStatement(tokens, cur_, end, state, STOP);
    if(succeeded) {
        if(state.direct) {
            throw ExecutionError("STOP command", ExecutionError::NOT_IN_DIRECT_STATE);
        }
        printf("STOP at line %d\n", state.current_line);
        state.direct = false;
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

    if(state.direct) {
        throw ExecutionError("GOTO command", ExecutionError::NOT_IN_DIRECT_STATE);
    }
    int goto_line = igr(cur++->value);

    if(cur >= end || ParseOne(tokens, cur, end, state, COLON)) {
        cur_ = cur;
        return state.goto_line = goto_line;
    }

    throw ParseError(tokens, ParseError::UNEXPECTED, cur - tokens.begin());
}

std::optional<VariableReference> ParseVariableReference(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state);

std::optional<Value> ParseParenExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state);

// term ::= number | STRING | variable-reference | function | paren-expression // returns Value
std::optional<Value> ParseTerm(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    if(cur_ >= end) { return {}; }

    if(auto results = ParseNumber(tokens, cur_, end)) {
        return *results;
    }

    if(cur_->type == STRING) {
        return cur_++->value;
    }

    if(auto results = ParseVariableReference(tokens, cur_ ,end, state)) {
        return EvaluateVariable(*results, state.variables);
    }

    if(auto results = ParseFunction(tokens, cur_, end, state)) {
        return *results;
    }

    if(auto results = ParseParenExpression(tokens, cur_, end, state)) {
        return *results;
    }

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

std::optional<Value> ParseExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state);

// paren-expression ::= OPEN_PAREN expression CLOSE_PAREN // returns Value
std::optional<Value> ParseParenExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;

    if(!ParseOne(tokens, cur, end, state, OPEN_PAREN)) {
        return {};
    }

    auto results = ParseExpression(tokens, cur, end, state);
    if(!results) {
        return {};
    }

    if(!ParseOne(tokens, cur, end, state, CLOSE_PAREN)) {
        return {};
    }

    cur_ = cur;
    return results;
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
            throw ParseError(tokens, ParseError::UNEXPECTED, cur - tokens.begin());
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

// let-statement ::= [LET] variable-reference EQUAL expression (COLON | end) // returns void
bool ParseLetStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;
    if(cur >= end) { return false; }

    bool committed_to_let = ParseOne(tokens, cur, end, state, LET).has_value();
    // If we saw "LET", we are committed from here, must emit parse error if can't match

    auto ref = ParseVariableReference(tokens, cur, end, state);
    if(!ref) {
        if(committed_to_let) {
            throw ParseError(tokens, "variable-reference", cur - tokens.begin());
        } else {
            return false;
        }
    }
    // Committed from here, must emit parse error if can't match

    if(!ParseOne(tokens, cur, end, state, EQUAL)) {
        throw ParseError(tokens, EQUAL, cur - tokens.begin());
    }

    auto value = ParseExpression(tokens, cur, end, state);
    if(!value) {
        throw ParseError(tokens, "expression", cur - tokens.begin());
    }
    if((ref->name[ref->name.size() - 1] == '$') && !is_str(*value)) {
        throw ExecutionError(ExecutionError::TYPE_MISMATCH);
    }
    if((ref->name[ref->name.size() - 1] != '$') && !is_num(*value)) {
        throw ExecutionError(ExecutionError::TYPE_MISMATCH);
    }

    if(cur->type == COLON) {
        cur++;
    }

    AssignVariable(*ref, *value, state.variables);

    cur_ = cur;
    return true;
}

// statement ::= ( print-statement | let-statement | input-statement | dim-statement | if-statement | for-statement | next-statement | on-statement | goto-statement | gosub-statement | wait-statement | width-statement | order-statement | read-statement | data-statement | deffn-statement | return-statement | end-statement | clear-statement | run-statement | stop-statement ) // returns void
void ParseStatement(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    // Either this succeeds or throws a parse error
    if(ParseEndStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("end\n");
        return;
    } else if(ParseClearStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("clear\n");
        return;
    } else if(ParseRunStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("run\n");
        return;
    } else if(ParseStopStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("stop\n");
        return;
    } else if(ParseGotoStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("goto %d\n", state.goto_line);
        return;
    } else if(ParsePrintStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("print\n");
        return;
    } else if(ParseLetStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("let\n");
        return;
#if 0
    // TODO
    } else if(ParseInputStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("input\n");
        return;
    } else if(ParseDimStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Dim\n");
        return;
    } else if(ParseIfStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("If\n");
        return;
    } else if(ParseForStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("For\n");
        return;
    } else if(ParseNextStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Next\n");
        return;
    } else if(ParseOnStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("On\n");
        return;
    } else if(ParseGosubStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Gosub\n");
        return;
    } else if(ParseWaitStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Wait\n");
        return;
    } else if(ParseWidthStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Width\n");
        return;
    } else if(ParseOrderStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Order\n");
        return;
    } else if(ParseReadStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Read\n");
        return;
    } else if(ParseDataStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Data\n");
        return;
    } else if(ParseDefStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Def\n");
        return;
    } else if(ParseReturnStatement(tokens, cur_, end, state)) {
        if(debug_statements) printf("Return\n");
        return;
#endif
    }
    throw ParseError(tokens, ParseError::UNEXPECTED, cur_ - tokens.begin());
}

// string-expression ::= expression that is a string // returns optional string
std::optional<std::string> ParseStringExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;
    if(auto results = ParseExpression(tokens, cur, end, state)) {
        if(is_str(*results)) {
            cur_ = cur;
            return str(*results);
        }
    }
    return {};
}


// integer-expression ::= expression that is an integer // returns optional integer
std::optional<int32_t> ParseIntegerExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;
    if(auto results = ParseExpression(tokens, cur, end, state)) {
        if(is_num(*results) && (num(*results) == igr(*results))) {
            cur_ = cur;
            return igr(*results);
        }
    }
    return {};
}

// numeric-expression ::= expression that is a number // returns optional double
std::optional<double> ParseNumericExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;
    if(auto results = ParseExpression(tokens, cur, end, state)) {
        if(is_num(*results)) {
            cur_ = cur;
            return num(*results);
        }
    }
    return {};
}

// variable-reference ::= identifier [OPEN_PAREN integer-expression {COMMA integer-expression} CLOSE_PAREN] // returns VariableReference
std::optional<VariableReference> ParseVariableReference(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    auto cur = cur_;

    auto identifier = ParseIdentifier(tokens, cur, end);
    if(!identifier) {
        return {};
    }

    if(cur >= end || !ParseOne(tokens, cur, end, state, OPEN_PAREN)) {
        cur_ = cur;
        return VariableReference(str(*identifier), {});
    }

    std::vector<int32_t> indices;

    auto integer = ParseIntegerExpression(tokens, cur, end, state);
    if(!integer) {
        return {};
    }
    indices.push_back(*integer);
    while(cur->type == COMMA) {
        cur++;
        integer = ParseIntegerExpression(tokens, cur, end, state);
        if(!integer) {
            return {};
        }
        indices.push_back(*integer);
    }

    if(cur >= end || !ParseOne(tokens, cur, end, state, CLOSE_PAREN)) {
        return {};
    }

    cur_ = cur;
    return VariableReference(str(*identifier), indices);
}

// statement-list ::= {statement} // returns void
void ParseStatementList(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    while(cur_ < end) {
        // Either this succeeds or throws a parse error
        ParseStatement(tokens, cur_, end, state);
    }
}

// Start ::= statement-list
void Parse(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{
    // Either this succeeds or throws a parse error
    ParseStatementList(tokens, cur_, end, state);
}

// ** I think these are correct:
// function
// variable-reference
// STRING
// number
// paren-expression ::= OPEN_PAREN expression CLOSE_PAREN // returns Value
// print-statement ::= PRINT {COMMA | SEMICOLON | expression} (COLON | END) // returns void
// unary-operation ::= {unary-op} term // evaluates using unary-ops, returns Value
// term ::= number | STRING | variable-reference | function | paren-expression // returns Value

// ** Not sure about these
// operation ::= expression (POWER | MULTIPLY | DIVIDE | PLUS | MINUS | LESS_THAN | GREATER_THAN | LESS_THAN_EQUAL | GREATER_THAN_EQUAL | EQUAL | NOT_EQUAL | AND | OR) expression // evaluates in correct order, returns Value
// operation is not referenced by any other rule
// expression ::= unary-operation | term // evaluates, returns Value

/*

I want highest priority to be evaluated first, like "1 + 2 * 3", evaluate "2 * 3" first, so that should be deepest descent.  so first level should match "term + expression" (1 + "2 * 3"), and second level should match "term * expression" (2 * "3"), and third level should match term ("3")
so e.g. expression := term (PLUS | MULTIPLY | POWER) expression ?
2 * 3 + 1?  Must match "2 * 3" + term" and then 
so e.g. expression := (expression PLUS expression) | (expression MULTIPLY expression) | (expression POWER expression) | term?
ParseOperation(... , operators = {POWER, MULTIPLY, PLUS})
    Value left, right;
    auto cur = cur_;
    if(operators.empty && (left == ParseTerm(...)) {
        cur_ = cur;
        return left;
    }
    higher_prec = operators;
    oper = operators.back();
    higher_prec.pop_back();
    // yikes lol
    if((left = ParseOperation(..., higher_prec)) || left = ParseTerm(...))) {
        if(ParseOne(..., oper)) { 
            if((right = ParseOperation(..., higher_prec)) || (right = ParseOperation(..., operators))) {
                Value result = do_operation(left, oper, right)
                cur_ = cur;
                return result;
            }
        }
    }
    cur = cur_
    if(Value term = ParseTerm(...)) {
        cur_ = cur;
        return term;
    }
    return {}

*/

Value DoOperation(const Value& left, TokenType oper, const Value& right)
{
    switch(oper) {
        case POWER:
            // TODO throw TypeMismatch if left and right not numbers
            return pow(num(left), num(right));
        case MULTIPLY:
            // TODO throw TypeMismatch if left and right not numbers
            return num(left) * num(right);
        case DIVIDE:
            // TODO throw TypeMismatch if left and right not numbers
            return num(left) / num(right);
        case PLUS:
            // TODO throw TypeMismatch if left and right not same type
            if(is_num(left)) {
                return num(left) + num(right);
            } else {
                return str(left) + str(right);
            }
        case MINUS:
            return num(left) - num(right);
        // TODO others
        default:
            // notreached
            return {0.0f};
            break;
    }
}

int indent = 0;
std::optional<Value> ParseOperation(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state, const std::vector<TokenType>& operators /* in decreasing precedence */)
{
    if(!operators.empty()) {
        auto cur = cur_;
        // yikes lol
        std::vector<TokenType> higher_prec = operators;
        TokenType oper = operators.back();
        higher_prec.pop_back();
        indent += 4;
        std::optional<Value> left = ParseOperation(tokens, cur, end, state, higher_prec);
        indent -= 4;
        if(left) printf("%*sparse left higher-precedence operation succeeded\n", indent, "");
        if(!left) {
            indent += 4;
            left = ParseUnaryOperation(tokens, cur, end, state);
            indent -= 4;
            if(left) printf("%*sparse left term succeeded\n", indent, "");
        }
        if(left) {
            indent += 4;
            if(ParseOne(tokens, cur, end, state, oper)) { 
                indent -= 4;
                printf("%*sfound %s\n", indent, "", TokenTypeToStringMap[oper]);
                indent += 4;
                std::optional<Value> right = ParseOperation(tokens, cur, end, state, higher_prec);
                indent -= 4;
                if(right) printf("%*sparse right higher-precedence operation succeeded\n", indent, "");
                if(!right) {
                    indent += 4;
                    right = ParseOperation(tokens, cur, end, state, operators);
                    indent -= 4;
                    if(right) printf("%*sparse right same operation succeeded\n", indent, "");
                }
                if(right) {
                    cur_ = cur;
                    return DoOperation(*left, oper, *right);
                }
            } else indent -= 4;
        }
    }

    auto cur = cur_;
    if(auto term = ParseUnaryOperation(tokens, cur, end, state)) {
        printf("%*sparse unary succeeded\n", indent, "");
        cur_ = cur;
        return term;
    }
    return {};
}

// expression ::= unary-operation // evaluates, returns Value
std::optional<Value> ParseExpression(const TokenList& tokens, TokenIterator& cur_, TokenIterator end, State& state)
{

    if(auto result = ParseOperation(tokens, cur_, end, state, {POWER, MULTIPLY, DIVIDE, PLUS, MINUS, LESS_THAN, GREATER_THAN, LESS_THAN_EQUAL, GREATER_THAN_EQUAL, EQUAL, NOT_EQUAL, AND, OR})) {
        return *result;
    }

    if(auto results = ParseUnaryOperation(tokens, cur_, end, state)) {
        return *results;
    }

    return {};
}

/* 
len-function ::= LEN OPEN_PAREN string-expression CLOSE_PAREN
val-function ::= VAL OPEN_PAREN string-expression CLOSE_PAREN
left-function ::= LEFT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
right-function ::= RIGHT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
mid-function ::= MID OPEN_PAREN string-expression COMMA numeric-expression [COMMA numeric-expression] CLOSE_PAREN
user-function ::= FN NUMBER_IDENTIFIER OPEN_PAREN numeric-expression [COMMA numeric-expression] CLOSE_PAREN
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


void ParseTest(const TokenList& tokens, TokenIterator& cur_, State& state)
{
    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();
        if(auto ref = ParseVariableReference(tokens, cur, end, state)) {
            printf("variable reference: %s", ref->name.c_str());
            if(!ref->indices.empty()) {
                printf("(%d", ref->indices.at(0));
                for(size_t i = 1; i <  ref->indices.size(); i++) {
                    printf(", %d", ref->indices.at(i));
                }
                printf(")");
            }
            printf("\n");
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseIntegerExpression(tokens, cur, end, state)) {
            printf("integer expression: %s\n", std::to_string(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseStringExpression(tokens, cur, end, state)) {
            printf("string expression: %s\n", value->c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseNumericExpression(tokens, cur, end, state)) {
            printf("numeric expression: %s\n", std::to_string(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseParenExpression(tokens, cur, end, state)) {
            printf("Paren expression: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseNumericFunction(tokens, cur, end, state)) {
            printf("function on numeric argument: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseFunction(tokens, cur, end, state)) {
            printf("function: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseExpression(tokens, cur, end, state)) {
            printf("Expression: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseTerm(tokens, cur, end, state)) {
            printf("term: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto value = ParseUnaryOperation(tokens, cur, end, state)) {
            printf("unary operation: %s\n", to_str(*value).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto identifier = ParseIdentifier(tokens, cur, end)) {
            printf("identifier \"%s\"\n", str(*identifier).c_str());
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto number = ParseNumber(tokens, cur, end)) {
            printf("number %f\n", num(*number));
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
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
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto integer = ParseInteger(tokens, cur, end)) {
            printf("integer %d\n", igr(*integer));
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
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
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto ttype = ParseUnaryOp(tokens, cur, end)) {
            printf("unary op %s\n", TokenTypeToStringMap[*ttype]);
            printf("    %zd tokens remaining \n", end - cur);
        }
    }

    {
        TokenIterator cur = cur_;
        TokenIterator end = tokens.end();

        if(auto ttype = ParseNumericFunctionName(tokens, cur, end)) {
            printf("numeric function name %d %s\n", *ttype, TokenTypeToStringMap[*ttype]);
            printf("    %zd tokens remaining \n", end - cur);
        }
    }
}


void EvaluateTokens(const TokenList& tokens, State& state)
{
    TokenIterator cur = tokens.begin();
    std::vector<Value> operands;
    std::vector<std::string> operators;
    std::vector<std::string> unary_operators;

    bool next_operator_is_unary = true;

    /* XXX for bringup */
    if(cur->type == TEST) {
        // XXX special token for testing; skips line number processing
        cur++;
        PrintTokenized(tokens);
        ParseTest(tokens, cur, state);
        exit(0);
    }

    if(cur >= tokens.end()) { throw ParseError(tokens); }
    if(cur->type == INTEGER) {
        int line_number = static_cast<int>(num(tokens.at(0).value));
        auto& line = state.program[line_number];
        std::copy(tokens.begin() + 1, tokens.end(), std::back_inserter(line));
        return;
    }

    TokenIterator end = tokens.end();
    Parse(tokens, cur, end, state);
    if(end - cur > 0) {
        throw ParseError(tokens, ParseError::TRAILING_TOKENS, cur - tokens.begin());
    }

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
    } else {
        printf("unimplemented command \"%s\"\n", command.c_str());
    }
#endif
}

// TODO handle GOSUB - this should probably be ExecuteNextStatement, so GOSUB can RETURN and continue on next statement on same line
void ExecuteNextLine(State& state)
{
    state.goto_line = -1;
    EvaluateTokens(state.program.at(state.current_line), state);

    if(state.direct) {
        // END command
        return;
    }

    if(state.goto_line == -1) {

        auto next_line = state.program.find(state.current_line);
        next_line++;
        if(next_line == state.program.end()) {
            // Last line of the program
            state.direct = true;
            return;
        }
        // printf("next line from %d yielded %d\n", state.current_line, next_line->first);
        state.current_line = next_line->first;

    } else {

        auto next_line = state.program.find(state.goto_line);
        if(next_line == state.program.end()) {
            throw ExecutionError(std::to_string(state.goto_line), ExecutionError::LINE_NOT_FOUND);
        }
        state.current_line = next_line->first;
    }
}

void StopExecution(State& state)
{
    if(!state.direct) {
        printf("Stopped at line %d\n", state.current_line);
    }
    state.direct = true;
}

int main(int argc, char **argv)
{
    State state;
    bool go = true;

    while(go) {
        try {
            if(state.direct) {
                static char line[512];
                if(fgets(line, sizeof(line), stdin) != nullptr) {
                    line[strlen(line) - 1] = '\0';
                    TokenList tokens;
                    try {
                        tokens = Tokenize(line);
                    } catch (const TokenizeError& e) {
                        switch(e.type) {
                            case TokenizeError::SYNTAX:
                                printf("syntax error at %d (\"%*s\")\n", e.position, std::min(5, (int)(strlen(line) - e.position)), line + e.position);
                                break;
                        }
                    }
                    EvaluateTokens(tokens, state);
                } else {
                    go = false;
                }
            } else {
                ExecuteNextLine(state);
            }
        } catch (const ParseError& e) {
            switch(e.type) {
                case ParseError::TRAILING_TOKENS:
                    printf("unrecognized trailing tokens\n");
                    break;
                case ParseError::UNEXPECTED_END:
                    printf("unexpected end of tokens while parsing\n");
                    break;
                case ParseError::UNEXPECTED:
                    printf("unexpected token while parsing\n");
                    break;
                case ParseError::EXPECTED_TOKEN:
                    printf("expected %s token at %d while parsing\n", TokenTypeToStringMap[e.expected_token_type], e.token);
                    break;
                case ParseError::EXPECTED_RULE:
                    printf("expected %s at %d while parsing\n", TokenTypeToStringMap[e.expected_token_type], e.token);
                    break;
            }
            PrintTokenized(e.tokens, e.token);
            StopExecution(state);
        } catch (const ExecutionError& e) {
            switch(e.type) {
                case ExecutionError::NOT_IN_RUN_STATE:
                    printf("RUN state required; %s\n", e.why.c_str());
                    break;
                case ExecutionError::NOT_IN_DIRECT_STATE:
                    printf("DIRECT state required; %s\n", e.why.c_str());
                    break;
                case ExecutionError::LINE_NOT_FOUND:
                    printf("Line %s not found\n", e.why.c_str());
                    break;
                case ExecutionError::VARIABLE_NOT_FOUND:
                    printf("Variable %s not found\n", e.why.c_str());
                    break;
                case ExecutionError::TYPE_MISMATCH:
                    printf("Type mismatch\n");
                    break;
            }
            StopExecution(state);
        } catch (const VariableDimensionError& e) {
            printf("array access for variable \"%s\" used %d dimensions, expected %d\n", e.var.c_str(), e.used, e.expected);
            StopExecution(state);
        } catch (const VariableReferenceBoundsError& e) {
            printf("variable \"%s\" referenced %d out of array bounds %d\n", e.var.c_str(),
                e.index, e.size);
            StopExecution(state);
        }
    }
}

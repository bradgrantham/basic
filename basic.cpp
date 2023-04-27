#include <string>
#include <variant>
#include <optional>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>

/*
TO DO
tokenize exponential notation
really there are string variables and then there are number variables which have INT and FLOAT values
    VariableValue should have a uniform type: string or number
    string array variables have no numbers in them
    number array variables have no strings in them
    no point in having variant with all three as foundation class?
    you already know number variables versus string variables
    Or just break apart array variables?
*/

/*
  * identifiers
  * keywords - greedy
  * are functions keywords or identifiers?
Slices through functionality
  * program line
  * run program line
  * immediate command
  * parse variable
  * change variable
  * call function
  * control flow

IDENTIFIER, INTEGER, FLOAT, STRING_LITERAL are all atoms, already tokenized
Buuut, what about IDENTIFIER OPEN_PAREN list CLOSE_PAREN?
* first token is an INTEGER - line is a program line, right is all statements
* COLON
* OPEN_PAREN - parse to next matching CLOSE_PAREN,
* reserved words (what about multiple reserved words?)
    DIM,
    LET,
    IF,
    THEN,
    ELSE,
    FOR,
    TO,
    STEP,
    NEXT,
    ON,
    GOTO,
    GOSUB,
    RETURN,
    PRINT,
    INPUT,
    END,
    WAIT,
    DEF, // But this is always followed by FN or in later basics INT, SNG, DBL, or STR
    WIDTH,
    CLEAR,
    ORDER,
    READ,
    DATA,
* separators
    COMMA,
    SEMICOLON,
* functions
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
    CHR,
    VAL,
* operators in reverse precedence order
    EQUAL,
    NOT_EQUAL,
    LESS_THAN,
    MORE_THAN,
    LESS_THAN_EQUAL,
    MORE_THAN_EQUAL,
    PLUS,
    MINUS,
    AND,
    OR,
    NOT,
    MODULO,
    MULTIPLY,
    DIVIDE,
    POWER,

DID:
identifier ::= NUMBER_IDENTIFIER | STRING_IDENTIFIER // returns Token
number ::= (FLOAT | INTEGER) // returns Token
unary-op ::= (PLUS | MINUS | NOT) // returns Token
integer-list ::= INTEGER (COMMA INTEGER)* // returns std::vector<int32_t>
numeric-function-name ::= ABS | ATN | COS | EXP | INT | LOG | RND | SGN | SIN | SQR | TAN | TAB | CHR | STR // returns Token
parameter-list ::= NUMBER_IDENTIFIER {COMMA NUMBER_IDENTIFIER} // returns std::vector<Token>
variable-reference ::= identifier [OPEN_PAREN numeric-expression {COMMA numeric-expression} CLOSE_PAREN] // returns VariableReference
paren-numeric-expression ::= OPEN_PAREN numeric-expression CLOSE_PAREN
term ::= {unary-op} (INTEGER | FLOAT | variable-reference | paren-numeric-expression) // evaluates using unary-ops, returns Value
special numeric-expression that is just term

DOING
exp-op ::= term POWER (term | exp-op) // evaluates, returns Value 

TODO:
product-op ::= term (MULTIPLY | DIVIDE | MODULO) (term | product-op | exp-op) // evaluates, returns Value
sum-op ::= term (PLUS | MINUS) (term | sum-op | product-op | exp-op) // evaluates, returns Value
relational-op ::= term (LESS_THAN | GREATER_THAN | LESS_THAN_EQUAL | MORE_THAN_EQUAL) (term | relational-op | sum-op | product-op | exp-op) // evaluates, returns Value
compare-op ::= term (EQUALS | NOT_EQUAL) (term | compare-op | relational-op | sum-op | product-op | exp-op) // evaluates, returns Value
logic-op ::= term (AND | OR) (term | logic-op | compare-op | relational-op | sum-op | product-op | exp-op) // evaluates, returns Value
numeric-expression ::= term | logic-op | compare-op | relational-op | sum-op | product-op | exp-op // evaluates, returns Value
function ::= (numeric-function-name OPEN_PAREN numeric-expression CLOSE_PAREN) |
             (LEN OPEN_PAREN string-expression CLOSE_PAREN) | 
             (VAL OPEN_PAREN string-expression CLOSE_PAREN) | 
             (LEFT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN) | 
             (RIGHT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN) | 
             (MID OPEN_PAREN string-expression COMMA numeric-expression [COMMA numeric-expression] CLOSE_PAREN)
             // evaluates, returns Value
expression ::= OPEN_PAREN expression CLOSE_PAREN | STRING_LITERAL | numeric-expression | function // evaluates, returns Value
expression-list ::= expression (COMMA expression)* // returns std::vector<Value>
variable-reference-list ::= variable-reference (COMMA | variable-reference)* // returns std::vector<VariableReference>
user-function ::= FUNCTION OPEN_PAREN expression-list CLOSE_PAREN // evaluates? returns Value
statement ::= PRINT (COMMA | SEMICOLON | expression)*
              [LET] variable-reference EQUAL expression
              INPUT [STRING_LITERAL SEMICOLON] variable-reference-list
              DIM identifier OPEN_PAREN integer-list CLOSE_PAREN
              IF expression THEN (INTEGER | statement) [ELSE (INTEGER | statement)]
              FOR NUMBER_IDENTIFIER EQUAL expression TO expression [STEP expression] // Only number identifiers?
              NEXT [NUMBER_IDENTIFIER]
              ON numeric-expression (GOTO | GOSUB) integer-list
              (GOTO | GOSUB) INTEGER
              WAIT numeric-expression
              WIDTH numeric-expression
              ORDER INTEGER
              READ variable-reference-list
              DATA expression-list
              DEF FUNCTION OPEN_PAREN parameter-list CLOSE_PAREN numeric-expression
              RETURN
              END
              CLEAR
              RUN
              STOP
              // does the work, returns void
              // some of these are not valid in direct mode, like INPUT
              // READ is discarded when not in direct
statement-list ::= statement (COLON statement)* // returns void
line ::= INTEGER statement-list EOL | statement-list EOL // returns void

vector<vector<TokenType>> PrecedenceTable =
{
    // what about NOT and unary MINUS?
    { POWER },
    { MULTIPLY, DIVIDE, MODULO, },
    { PLUS, MINUS, },
    { LESS_THAN, GREATER_THAN, LESS_THAN_EQUAL, MORE_THAN_EQUAL },
    { EQUALS, NOT_EQUAL},
    { AND, OR },
    // FUNCTIONS
};

// require OPEN_PAREN, then expressions separated by COMMA, then CLOSE_PAREN
std::set<TokenType> Functions =
{
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
    CHR,
    LEFT,
    RIGHT,
    MID,
    LEN,
    STR,
    TAB,
    VAL,
};

Control flow or reserved words
    IF expression,
    THEN statements,
    ELSE statements,
    FOR identifier EQUAL expression, TO expression, STEP expression
    NEXT,
    GOTO expression, GOSUB expression,
    RETURN
    END,
    ON expression GOSUB list,
    ON expression GOTO list,
    DEF FN identifier OPEN_PAREN identifier list CLOSE_PAREN expression (using identifiers),
    DIM identifier OPEN_PAREN INTEGER CLOSE_PAREN,
    WAIT expression, // hundredths of a second
    WIDTH expression, // ignore?
    DATA list of expressions,
    READ list of identifiers,
    ORDER expression
    CLEAR
    PRINT {SEMICOLON, COMMA, expression} ... [;]
    INPUT [STRING_LITERAL SEMICOLON] expression list

Unknown
    identifier EQUAL expression

COLON (statements)
numbered lines
*/


enum class TokenType
{
    STRING_IDENTIFIER,
    NUMBER_IDENTIFIER,
    INTEGER,
    FLOAT,
    STRING_LITERAL,
    REMARK,  		 // Remark (comment) starting with REM keyword
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
    NOT_EQUAL,
    LESS_THAN,
    MORE_THAN,
    LESS_THAN_EQUAL,
    MORE_THAN_EQUAL,
    OPEN_PAREN,
    CLOSE_PAREN,
    EQUAL,
    PLUS,
    MINUS,
    MULTIPLY,
    DIVIDE,
    MODULO,
    POWER,
    COMMA,
    COLON,
    SEMICOLON,
    AND,
    OR,
    NOT,
    LEFT,
    RIGHT,
    MID,
    LEN,
    STR,
    TAB,
    VAL,
    WAIT,
    DEF,
    FN,
    CHR,
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
    {"<>", TokenType::NOT_EQUAL},
    {"<=", TokenType::LESS_THAN_EQUAL},
    {">=", TokenType::MORE_THAN_EQUAL},
    {">", TokenType::MORE_THAN_EQUAL},
    {"<", TokenType::LESS_THAN},
    {"(", TokenType::OPEN_PAREN},
    {")", TokenType::CLOSE_PAREN},
    {"=", TokenType::EQUAL},
    {"+", TokenType::PLUS},
    {"-", TokenType::MINUS},
    {"*", TokenType::MULTIPLY},
    {"/", TokenType::DIVIDE},
    {"%", TokenType::MODULO},
    {"^", TokenType::POWER},
    {",", TokenType::COMMA},
    {":", TokenType::COLON},
    {";", TokenType::SEMICOLON},
    {"DIM", TokenType::DIM},
    {"LET", TokenType::LET},
    {"IF", TokenType::IF},
    {"THEN", TokenType::THEN},
    {"ELSE", TokenType::ELSE},
    {"FOR", TokenType::FOR},
    {"TO", TokenType::TO},
    {"STEP", TokenType::STEP},
    {"NEXT", TokenType::NEXT},
    {"GOTO", TokenType::GOTO},
    {"GOSUB", TokenType::GOSUB},
    {"RETURN", TokenType::RETURN},
    {"PRINT", TokenType::PRINT},
    {"INPUT", TokenType::INPUT},
    {"END", TokenType::END},
    {"ABS", TokenType::ABS},
    {"ATN", TokenType::ATN},
    {"COS", TokenType::COS},
    {"EXP", TokenType::EXP},
    {"INT", TokenType::INT},
    {"LOG", TokenType::LOG},
    {"RND", TokenType::RND},
    {"SGN", TokenType::SGN},
    {"SIN", TokenType::SIN},
    {"SQR", TokenType::SQR},
    {"TAN", TokenType::TAN},
    {"AND", TokenType::AND},
    {"OR", TokenType::OR},
    {"NOT", TokenType::NOT},
    {"LEFT$", TokenType::LEFT},
    {"RIGHT$", TokenType::RIGHT},
    {"MID$", TokenType::MID},
    {"STR$", TokenType::STR},
    {"LEN", TokenType::LEN},
    {"TAB", TokenType::TAB},
    {"VAL", TokenType::VAL},
    {"WAIT", TokenType::WAIT},
    {"DEF", TokenType::DEF},
    {"FN", TokenType::FN},
    {"CHR$", TokenType::CHR},
    {"WIDTH", TokenType::WIDTH},
    {"CLEAR", TokenType::CLEAR},
    {"ON", TokenType::ON},
    {"READ", TokenType::READ},
    {"ORDER", TokenType::ORDER},
    {"DATA", TokenType::DATA},
    {"RUN", TokenType::RUN},
    {"STOP", TokenType::STOP},
};

std::unordered_map<TokenType, const char *> TokenTypeToStringMap =
{
    {TokenType::EQUAL, "="},
    {TokenType::NOT_EQUAL, "<>"},
    {TokenType::LESS_THAN_EQUAL, "<="},
    {TokenType::MORE_THAN_EQUAL, ">="},
    {TokenType::LESS_THAN, "<"},
    {TokenType::MORE_THAN, ">"},
    {TokenType::OPEN_PAREN, "("},
    {TokenType::CLOSE_PAREN, ")"},
    {TokenType::PLUS, "+"},
    {TokenType::MINUS, "-"},
    {TokenType::MULTIPLY, "*"},
    {TokenType::DIVIDE, "/"},
    {TokenType::MODULO, "%"},
    {TokenType::POWER, "^"},
    {TokenType::COMMA, ","},
    {TokenType::COLON, ":"},
    {TokenType::SEMICOLON, ";"},
    {TokenType::DIM, "DIM"},
    {TokenType::LET, "LET"},
    {TokenType::IF, "IF"},
    {TokenType::THEN, "THEN"},
    {TokenType::ELSE, "ELSE"},
    {TokenType::FOR, "FOR"},
    {TokenType::TO, "TO"},
    {TokenType::STEP, "STEP"},
    {TokenType::NEXT, "NEXT"},
    {TokenType::GOTO, "GOTO"},
    {TokenType::GOSUB, "GOSUB"},
    {TokenType::RETURN, "RETURN"},
    {TokenType::PRINT, "PRINT"},
    {TokenType::INPUT, "INPUT"},
    {TokenType::END, "END"},
    {TokenType::ABS, "ABS"},
    {TokenType::ATN, "ATN"},
    {TokenType::COS, "COS"},
    {TokenType::EXP, "EXP"},
    {TokenType::INT, "INT"},
    {TokenType::LOG, "LOG"},
    {TokenType::RND, "RND"},
    {TokenType::SGN, "SGN"},
    {TokenType::SIN, "SIN"},
    {TokenType::SQR, "SQR"},
    {TokenType::TAN, "TAN"},
    {TokenType::AND, "AND"},
    {TokenType::OR, "OR"},
    {TokenType::NOT, "NOT"},
    {TokenType::LEFT, "LEFT$"},
    {TokenType::RIGHT, "RIGHT$"},
    {TokenType::MID, "MID$"},
    {TokenType::STR, "STR$"},
    {TokenType::LEN, "LEN"},
    {TokenType::TAB, "TAB"},
    {TokenType::VAL, "VAL"},
    {TokenType::WAIT, "WAIT"},
    {TokenType::DEF, "DEF"},
    {TokenType::FN, "FN"},
    {TokenType::CHR, "CHR$"},
    {TokenType::WIDTH, "WIDTH"},
    {TokenType::CLEAR, "CLEAR"},
    {TokenType::ON, "ON"},
    {TokenType::READ, "READ"},
    {TokenType::ORDER, "ORDER"},
    {TokenType::DATA, "DATA"},
    {TokenType::RUN, "RUN"},
    {TokenType::STOP, "STOP"},
};

typedef std::variant<std::string, int32_t, double> Value;

struct Token
{
    public:
        TokenType type;
        std::optional<Value> value;

        Token(TokenType type) : type(type) {}
        Token(TokenType type, const std::string& value) : type(type), value(value) {}
        Token(int32_t value) : type(TokenType::INTEGER), value(value) {}
        Token(double value) : type(TokenType::FLOAT), value(value) {}
};
typedef std::vector<Token> TokenList;
typedef TokenList::const_iterator TokenIterator;

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

struct VariableNotFoundError
{
    std::string var;
    VariableNotFoundError(const std::string var) :
        var(var)
    {}
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
                    tokens.push_back(Token(TokenType::STRING_IDENTIFIER, pending));
                } else {
                    tokens.push_back(Token(TokenType::NUMBER_IDENTIFIER, pending));
                }
                pending.clear();
                pending_started = std::string::npos;
            }
        }
    };

    for (size_t index = 0; index < line.size();) {
        if(str_toupper(line.substr(index, 3)) == "REM") {
            flush_pending();
            tokens.push_back(Token(TokenType::REMARK, line.substr(index + 3)));
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
            tokens.push_back(Token(TokenType::STRING_LITERAL, str));
            continue;
        }

        auto result = [&]() -> std::optional<std::pair<size_t, TokenType>>{
            // XXX Could probably make a custom compare that would fit std::map
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

void print_tokenized(const TokenList& tokens)
{
    printf("%zd tokens: ", tokens.size());
    for(const auto& t: tokens) {
        switch(t.type) {
            case TokenType::STRING_IDENTIFIER:
            case TokenType::NUMBER_IDENTIFIER:
            {
                auto v = t.value.value();
                printf("%s ", std::get<std::string>(v).c_str());
                break;
            }
            case TokenType::FLOAT: {
                auto v = t.value.value();
                printf("%f ", std::get<double>(v));
                break;
            }
            case TokenType::INTEGER: {
                auto v = t.value.value();
                printf("%d ", std::get<int32_t>(v));
                break;
            }
            case TokenType::REMARK: {
                auto v = t.value.value();
                printf("REM%s ", std::get<std::string>(v).c_str());
                break;
            }
            case TokenType::STRING_LITERAL: {
                auto v = t.value.value();
                printf("\"%s\" ", std::get<std::string>(v).c_str());
                break;
            }
            default: {
                printf("%s ", TokenTypeToStringMap[t.type]);
                break;
            }
        }
    }
    printf("\n");
}

std::string str(const Value& v) { return std::get<std::string>(v); }
double dbl(const Value& v) { return std::get<double>(v); }
int32_t igr(const Value& v) { return std::get<int32_t>(v); }
bool is_str(const Value& v) { return std::holds_alternative<std::string>(v); }
bool is_dbl(const Value& v) { return std::holds_alternative<double>(v); }
bool is_igr(const Value& v) { return std::holds_alternative<int32_t>(v); }

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

struct VariableReference
{
    std::string name;
    std::vector<int32_t> indices;
    VariableReference(const std::string& name) : name(name)
    {}
    VariableReference(const std::string& name, const std::vector<int32_t>& indices) : name(name), indices(indices)
    {}
};

typedef std::unordered_map<std::string, VariableValue> VariableMap;
typedef std::optional<Value> Result;
typedef std::map<int32_t, TokenList> StoredProgram;

struct State {
    VariableMap variables;
    StoredProgram program;
    bool direct_mode{true};
};

struct TypeMismatchError { };
struct MissingCloseParenError { };

Value EvaluateVariable(const VariableReference& ref, const VariableMap& variables)
{
    auto iter = variables.find(ref.name);
    if(iter == variables.end()) {
        throw VariableNotFoundError(ref.name);
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

template <class Op>
Value EvaluateUnary(Value v, Op op)
{
    if(is_str(v)) {
        throw TypeMismatchError();
    }
    if(is_dbl(v)) {
        return dbl(v);
    } else {
        return igr(v);
    }
}

template <class Op>
Value EvaluateBinary(Value l, Value r, Op op)
{
    if(is_str(r)) {
        throw TypeMismatchError();
    }
    if(is_str(l)) {
        throw TypeMismatchError();
    }
    if(is_dbl(l) && is_dbl(r)) {
        return op(dbl(l), dbl(r));
    } else if(is_dbl(l)) {
        return op(dbl(l), igr(r));
    } else if(is_dbl(r)) {
        return op(igr(l), dbl(r));
    } else {
        return op(igr(l), igr(r));
    }
}

// For every Evaluate function operating on a pair of TokenIterators,
// if evaluation is successful, begin is incremented to the end of
// the sequence (the beginning of the next sequence).

std::optional<Token> EvaluateIdentifier(TokenIterator& begin, TokenIterator& end)
{
    if(begin == end) { return std::nullopt; }

    if(begin->type == TokenType::NUMBER_IDENTIFIER ||
        begin->type == TokenType::STRING_IDENTIFIER) {
        return *begin++;
    }
    return std::nullopt;
}

std::optional<Token> EvaluateNumber(TokenIterator& begin, TokenIterator& end)
{
    if(begin == end) { return std::nullopt; }

    if(begin->type == TokenType::FLOAT ||
        begin->type == TokenType::INTEGER) {
        return *begin++;
    }
    return std::nullopt;
}

std::optional<Token> EvaluateUnaryOperator(TokenIterator& begin, TokenIterator& end)
{
    if(begin == end) { return std::nullopt; }

    if(begin->type == TokenType::PLUS ||
        begin->type == TokenType::NOT ||
        begin->type == TokenType::MINUS) {
        return *begin++;
    }
    return std::nullopt;
}

std::optional<std::vector<int32_t>> EvaluateIntegerList(TokenIterator& begin, TokenIterator& end)
{
    if(begin == end) { return std::nullopt; }

    std::vector<int32_t> ints;
    if(begin->type != TokenType::INTEGER) {
        return std::nullopt;
    }
    ints.push_back(std::get<int32_t>(begin->value.value()));

    begin++;

    while((begin + 1 < end) &&
        (begin->type == TokenType::COMMA && (begin + 1)->type == TokenType::INTEGER))
    {
        ints.push_back(std::get<int32_t>((begin + 1)->value.value()));
        begin += 2;
    }

    return ints;
}

std::optional<std::vector<Token>> EvaluateParameterList(TokenIterator& begin, TokenIterator& end)
{
    if(begin == end) { return std::nullopt; }

    std::vector<Token> parameters;
    if(begin->type != TokenType::NUMBER_IDENTIFIER) {
        return std::nullopt;
    }
    parameters.push_back(*begin);
    begin++;

    while((begin + 1 < end) && 
        (begin->type == TokenType::COMMA && (begin + 1)->type == TokenType::NUMBER_IDENTIFIER))
    {
        parameters.push_back(*(begin + 1));
        begin += 2;
    }

    return parameters;
}

std::optional<Token> EvaluateOneToken(TokenIterator begin, TokenIterator end, const std::set<TokenType>& valid)
{
    if(begin == end) { return std::nullopt; }

    if(valid.count(begin->type) > 0) {
        return *begin++;
    }
    return std::nullopt;
}

std::optional<Value> EvaluateNumericExpression(TokenIterator& begin, TokenIterator& end, State& state);

// variable-reference ::= identifier [OPEN_PAREN numeric-expression [COMMA numeric-expression] CLOSE_PAREN] // returns VariableReference
std::optional<VariableReference> EvaluateVariableReference(TokenIterator& begin_, TokenIterator& end, State& state)
{
    auto begin = begin_;
    auto resultid = EvaluateIdentifier(begin, end);
    if(!resultid) {
        return std::nullopt;
    }
    auto identifier = *resultid;

    VariableReference ref(std::get<std::string>(identifier.value.value()));

    if((begin == end) || (begin->type != TokenType::OPEN_PAREN)) {
        begin_ = begin;
        return ref;
    }
    begin++;

    auto& indices = ref.indices;
    auto result = EvaluateNumericExpression(begin, end, state);
    if(!result) {
        return std::nullopt;
    }
    auto v = result.value();
    if(is_igr(v)) {
        indices.push_back(igr(v));
    } else {
        indices.push_back(static_cast<int32_t>(trunc(dbl(v))));
    }

    while(begin < end && begin->type != TokenType::CLOSE_PAREN)
    {
        if(begin->type != TokenType::COMMA) {
            return std::nullopt;
        }

        begin++;

        result = EvaluateNumericExpression(begin, end, state);
        if(!result) {
            return std::nullopt;
        }

        v = result.value();
        if(is_igr(v)) {
            indices.push_back(igr(v));
        } else {
            indices.push_back(static_cast<int32_t>(trunc(dbl(v))));
        }
    }

    if(begin == end) {
        throw MissingCloseParenError();
    }

    begin++;
    begin_ = begin;
    return ref;
}

// paren-numeric-expression ::= OPEN_PAREN numeric-expression CLOSE_PAREN
std::optional<Value> EvaluateParenNumericExpression(TokenIterator& begin_, TokenIterator& end, State& state)
{
    auto begin = begin_;

    if(begin == end) { return std::nullopt; }

    if(begin->type != TokenType::OPEN_PAREN) {
        return std::nullopt;
    }
    begin++;
    auto result = EvaluateNumericExpression(begin, end, state);
    if(!result) {
        return std::nullopt;
    }
    auto v = result.value();

    if(begin == end) {
        throw MissingCloseParenError();
    }
    if(begin->type != TokenType::CLOSE_PAREN) {
        return std::nullopt;
    }

    begin++;
    begin_ = begin;
    return v;
}

std::optional<Value> EvaluateTerm(TokenIterator& begin_, TokenIterator& end, State& state)
{
    auto begin = begin_;
    std::vector<TokenType> operators;
    while(auto unary_op = EvaluateUnaryOperator(begin, end)) {
        operators.push_back(unary_op->type);
    }

    if(begin == end) { return std::nullopt; }

    if(begin->type == TokenType::INTEGER) {
        int32_t v = std::get<int32_t>(begin->value.value());
        for(auto it = operators.rbegin(); it != operators.rend(); it++) {
            switch(*it) {
                case TokenType::PLUS: break;
                case TokenType::MINUS: v = -v; break;
                case TokenType::NOT: v = -1 - v; break;
                default: break;
            }
        }
        begin_ = begin + 1;
        return v;
    }

    if(begin->type == TokenType::FLOAT) {
        double v = std::get<double>(begin->value.value());
        for(auto it = operators.rbegin(); it != operators.rend(); it++) {
            switch(*it) {
                case TokenType::PLUS: break;
                case TokenType::MINUS: v = -v; break;
                case TokenType::NOT: v = -1 - static_cast<int>(trunc(v)); break;
                default: break;
            }
        }
        begin_ = begin + 1;
        return v;
    }

    Value v;
    auto result = EvaluateVariableReference(begin, end, state);

    if(result) {

        auto ref = result.value();
        v = EvaluateVariable(ref, state.variables);

    } else { 

        auto result = EvaluateParenNumericExpression(begin, end, state);
        if(!result) {
            return std::nullopt;
        }
        v = result.value();
    }

    if(!is_igr(v) && !is_dbl(v)) {
        throw TypeMismatchError();
    }
    for(auto it = operators.rbegin(); it != operators.rend(); it++) {
        switch(*it) {
            case TokenType::PLUS:
                break;
            case TokenType::MINUS:
                if(is_igr(v)) {
                    v = - igr(v);
                } else {
                    v = - dbl(v);
                }
                break;
            case TokenType::NOT:
                if(is_igr(v)) {
                    v = -1 - igr(v);
                } else {
                    v = -1 - static_cast<int32_t>(trunc(dbl(v)));
                }
                break;
            default: break;
        }
    }
    begin_ = begin;
    return v;
}

std::optional<Value> EvaluateExpOp(TokenIterator& begin_, TokenIterator& end, State& state)
{

}

// XXX special version limited to just "term"
std::optional<Value> EvaluateNumericExpression(TokenIterator& begin, TokenIterator& end, State& state)
{
    auto result = EvaluateTerm(begin, end, state);
    if(result) {
        return result;
    }

    // XXX Evaluate all others too
    return std::nullopt;
}
// numeric-expression ::= term | logic-op | compare-op | relational-op | sum-op | product-op | exp-op // evaluates, returns Value

void EvaluateTokens(const TokenList& tokens, State& state)
{
    if(tokens.size() == 0) {
        // Should evaluate to nothing.
        return;
    }
    if(false && tokens[0].type == TokenType::INTEGER) {
        // Line number - a program line
        auto line_number = std::get<int32_t>(tokens[0].value.value());
        auto line = TokenList(tokens.cbegin() + 1, tokens.cend());
        state.program[line_number] = line;
        return;
    }
    if(tokens[0].type == TokenType::REMARK) {
        return;
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateIdentifier(begin, end);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto token = result.value();
            printf("identifier %s\n", std::get<std::string>(token.value.value()).c_str());
        } else {
            printf("failed identifier\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateNumber(begin, end);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto token = result.value();
            auto v = token.value.value();
            if(is_igr(v)) {
                printf("integer %d\n", igr(v));
            } else {
                printf("float %f\n", dbl(v));
            }
        } else {
            printf("failed number\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateUnaryOperator(begin, end);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto token = result.value();
            if(token.type == TokenType::MINUS) {
                printf("MINUS\n");
            } else if(token.type == TokenType::PLUS) {
                printf("PLUS\n");
            } else {
                printf("NOT\n");
            }
        } else {
            printf("failed unary operator\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateIntegerList(begin, end);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto ints = result.value();
            for(auto i: ints) {
                printf("%d ", i);
            }
            printf("\n");
        } else {
            printf("failed integer list\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateParameterList(begin, end);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto tokens = result.value();
            for(const auto& t: tokens) {
                printf("%s ", std::get<std::string>(t.value.value()).c_str());
            }
            printf("\n");
        } else {
            printf("failed parameter list\n");
        }
    }

    std::set<TokenType> numeric_function_names { TokenType::ABS, TokenType::ATN, TokenType::COS, TokenType::EXP, TokenType::INT, TokenType::LOG, TokenType::RND, TokenType::SGN, TokenType::SIN, TokenType::SQR, TokenType::TAN, TokenType::TAB, TokenType::CHR, TokenType::STR };
    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateOneToken(begin, end, numeric_function_names);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto token = result.value();
            printf("numeric function %s\n", TokenTypeToStringMap[token.type]);
        } else {
            printf("failed numeric function\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateVariableReference(begin, end, state);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto ref = result.value();
            printf("variable reference %s ", ref.name.c_str());
            for(auto i: ref.indices) {
                printf("%d ", i);
            }
            printf("\n");
        } else {
            printf("failed variable reference\n");
        }
    }

    {
        auto begin = tokens.cbegin();
        auto end = tokens.cend();
        auto result = EvaluateNumericExpression(begin, end, state);
        printf("%zd consumed : ", begin - tokens.cbegin());
        if(result) {
            auto v = result.value();
            if(is_igr(v)) {
                printf("expression integer %d\n", igr(v));
            } else {
                printf("expression float %f\n", dbl(v));
            }
        } else {
            printf("failed numeric expression\n");
        }
    }
}

#if 0

struct InvalidLValueError { };

template <class Op>
struct ASTString1Param : public ASTNode
{
    ASTNodePtr node;
    Op op;
    ASTString1Param(ASTNodePtr node, Op op) :
        node(std::move(node)),
        op(op)
    {}
    virtual ASTValue evaluateR(VariableMap& variables, StoredProgram& program) override
    {
        Value v = node->evaluateR(variables, program).value();
        if(!std::holds_alternative<std::string>(v)) {
            throw TypeMismatchError();
        }
        return op(std::get<std::string>(v));
    }
    virtual ~ASTString1Param() {}
};

template <class Op>
struct ASTString2Param : public ASTNode
{
    ASTNodePtr param1;
    ASTNodePtr param2;
    Op op;
    ASTString2Param(ASTNodePtr param1, ASTNodePtr param2, Op op) :
        param1(std::move(param1)),
        param2(std::move(param2)),
        op(op)
    {}
    virtual ASTValue evaluateR(VariableMap& variables, StoredProgram& program) override
    {
        Value value1 = param1->evaluateR(variables, program).value();
        Value value2 = param2->evaluateR(variables, program).value();
        if(!std::holds_alternative<std::string>(value1)) {
            printf("value 1 not string\n");
            throw TypeMismatchError();
        }
        if(!std::holds_alternative<std::int32_t>(value2)) {
            printf("value 2 not int32_t\n");
            throw TypeMismatchError();
        }
        return op(std::get<std::string>(value1), std::get<std::int32_t>(value2));
    }
    virtual ~ASTString2Param() {}
};

#endif

std::string to_string(const Result& r)
{
    if(r.has_value()) {
        auto v = r.value();
        if(is_igr(v)) {
            return std::to_string(igr(v));
        } else if(is_dbl(v)) {
            return std::to_string(dbl(v));
        } else if(is_igr(v)) {
            return str(v).c_str();
        } else {
            abort();
        }
    } else {
        return "NoValue";
    }
}

int main(int argc, char **argv)
{
    char line[1024];
    std::set<std::string> identifiers;

    State state;
    VariableValue val{123.0, {10}, 0.0};
    state.variables.insert({"A", val});
    VariableValue val2{"Hello", {10}, ""};
    state.variables.insert({"B$", val2});

    if(true) {
        std::set<std::string> identifiers;
        while(fgets(line, sizeof(line), stdin) != NULL) {
            line[strlen(line) - 1] = '\0';
            try {
                auto tokens = Tokenize(line);
                print_tokenized(tokens);
                for(const auto& token: tokens) {
                    if(token.type == TokenType::STRING_IDENTIFIER || token.type == TokenType::NUMBER_IDENTIFIER) {
                        auto value = token.value.value();
                        identifiers.insert(std::get<std::string>(value).c_str());
                    }
                }
                EvaluateTokens(tokens, state);
            } catch (const TokenizeError& e) {
                switch(e.type) {
                    case TokenizeError::SYNTAX:
                        printf("syntax error at %d (\"%*s\")\n", e.position, std::min(5, (int)(strlen(line) - e.position)), line + e.position);
                }
            } catch (const TypeMismatchError& e) {
                printf("expected a number, encountered a string\n");
            } catch (const VariableNotFoundError& e) {
                printf("unknown variable \"%s\"\n", e.var.c_str());
            } catch (const VariableDimensionError& e) {
                printf("array access for variable \"%s\" used %d dimensions, expected %d\n", e.var.c_str(), e.used, e.expected);
            } catch (const VariableReferenceBoundsError& e) {
                printf("variable \"%s\" referenced %d out of array bounds %d\n", e.var.c_str(),
                    e.index, e.size);
            }
        }
        if(false) {
            printf("identifiers:\n");
            for(const auto &id: identifiers) {
                printf("%s\n", id.c_str());
            }
        }
    }
}

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
Must figure out soon
IDENTIFIER - different tokens for string and number IDENTIFIER?
DEF FN A - should FN not be a reserved word, and just require all functions to start with "FN"?
        FN_IDENTIFIER, NUMBER_IDENTIFIER, and STRING_IDENTIFIER?
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

// need separate string and number identifiers and expressions?
line ::= INTEGER statement-list EOL | statement-list EOL
statement-list = statement (COLON statement)*
statement ::= PRINT (COMMA | SEMICOLON | expression)*
              [LET] variable-reference EQUAL expression
              INPUT [STRING_LITERAL SEMICOLON] variable-reference (COMMA | variable-reference)*
              DIM IDENTIFIER OPEN_PAREN INTEGER CLOSE_PAREN
              IF expression THEN (INTEGER | statement) [ELSE statement]
              FOR IDENTIFIER EQUAL expression TO expression [STEP expression] // Only number identifiers?
              NEXT [IDENTIFIER]
              ...
              ON numeric-expression (GOTO | GOSUB) INTEGER (COMMA INTEGER)*
              GOTO INTEGER
              GOSUB INTEGER
              RETURN
              END
              CLEAR
              WAIT numeric-expression
              WIDTH numeric-expression
              READ variable-reference
              DATA expression (COMMA expression)*
              DEF FN IDENTIFIER OPEN_PAREN IDENTIFIER (COMMA IDENTIFIER)* CLOSE_PAREN expression
variable-reference ::= IDENTIFIER [OPEN_PAREN numeric-expression CLOSE_PAREN]
expression ::= OPEN_PAREN expression CLOSE_PAREN | STRING_LITERAL | numeric-expression | function
function ::= ABS OPEN_PAREN numeric-expression CLOSE_PAREN
             ATN OPEN_PAREN numeric-expression CLOSE_PAREN
             COS OPEN_PAREN numeric-expression CLOSE_PAREN
             EXP OPEN_PAREN numeric-expression CLOSE_PAREN
             INT OPEN_PAREN numeric-expression CLOSE_PAREN
             LOG OPEN_PAREN numeric-expression CLOSE_PAREN
             RND OPEN_PAREN numeric-expression CLOSE_PAREN
             SGN OPEN_PAREN numeric-expression CLOSE_PAREN
             SIN OPEN_PAREN numeric-expression CLOSE_PAREN
             SQR OPEN_PAREN numeric-expression CLOSE_PAREN
             TAN OPEN_PAREN numeric-expression CLOSE_PAREN
             TAB OPEN_PAREN numeric-expression CLOSE_PAREN
             CHR OPEN_PAREN numeric-expression CLOSE_PAREN
             STR OPEN_PAREN numeric-expression CLOSE_PAREN
             LEN OPEN_PAREN string-expression CLOSE_PAREN
             LEFT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
             RIGHT OPEN_PAREN string-expression COMMA numeric-expression CLOSE_PAREN
             MID OPEN_PAREN string-expression COMMA numeric-expression [COMMA numeric-expression] CLOSE_PAREN
numeric-expression ::= INTEGER | FLOAT | (unary-op numeric-expression) (binary-op | numeric-expression)*
unary-op ::= (PLUS | MINUS | NOT)
binary-op ::= PLUS
              MINUS
              MULTIPLY
              DIVIDE
              POWER
              MODULO
              LESS_THAN
              GREATER_THAN
              LESS_THAN_EQUAL
              MORE_THAN_EQUAL
              EQUALS
              NOT_EQUAL
              AND
              OR
number ::= (FLOAT | INTEGER)

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
    IDENTIFIER,
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
    WAIT,
    DEF,
    FN,
    CHR,
    WIDTH,
    CLEAR,
    ON,
    READ,
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
    {"WAIT", TokenType::WAIT},
    {"DEF", TokenType::DEF},
    {"FN", TokenType::FN},
    {"CHR$", TokenType::CHR},
    {"WIDTH", TokenType::WIDTH},
    {"CLEAR", TokenType::CLEAR},
    {"ON", TokenType::ON},
    {"READ", TokenType::READ},
    {"DATA", TokenType::DATA},
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
    {TokenType::WAIT, "WAIT"},
    {TokenType::DEF, "DEF"},
    {TokenType::FN, "FN"},
    {TokenType::CHR, "CHR$"},
    {TokenType::WIDTH, "WIDTH"},
    {TokenType::CLEAR, "CLEAR"},
    {TokenType::ON, "ON"},
    {TokenType::READ, "READ"},
    {TokenType::DATA, "DATA"},
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
                for(int i = 0; i < pending.size(); i++) {
                    char c = pending[i];
                    if(!isalnum(c) && c != '_' && c != '$') {
                        throw TokenizeError(TokenizeError::SYNTAX, pending_started + i);
                    }
                }
                tokens.push_back(Token(TokenType::IDENTIFIER, pending));
                pending.clear();
                pending_started = std::string::npos;
            }
        }
    };

    for (int index = 0; index < line.size();)
    {
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

        if (result.has_value())
        {
            flush_pending();
            auto size = result.value().first;
            auto token = result.value().second;
            tokens.push_back(Token(token));
            index += size;
        }
        else
        {
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
            case TokenType::IDENTIFIER: {
                auto v = t.value.value();
                printf("%s ", std::get<std::string>(v).c_str());
                break;
            }
            case TokenType::FLOAT: {
                auto v = t.value.value();
                printf("%.f ", std::get<double>(v));
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

typedef std::variant<Value, std::vector<Value>> MultiValue;
typedef std::unordered_map<std::string, MultiValue> VariableMap;
// Vector is indices into dimensioned variables
typedef std::pair<std::string, int> VariableReference;
typedef std::optional<Value> Result;
typedef std::map<int32_t, TokenList> StoredProgram;

struct TypeMismatchError { };

Value EvaluateVariable(const VariableReference& ref, const VariableMap& variables)
{
    auto iter = variables.find(ref.first);
    if(iter == variables.end()) {
        throw VariableNotFoundError(ref.first);
    }
    auto value = iter->second;
    if(std::holds_alternative<std::vector<Value>>(value)) {
        return std::get<std::vector<Value>>(value)[ref.second];
    }
    return std::get<Value>(value);
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

#if 0
    TokenList value_stack;
    TokenList operator_stack;
    for(const auto& t: tokens) {
        switch(t.type) {
            case TokenType::INTEGER:
            case TokenType::FLOAT:
            case TokenType::STRING_LITERAL:
            {
                if(!operator_stack.empty()) {
                    auto op = operator_stack.back();
                    operator_stack.pop_back();
                    switch(op.type) {
                        case TokenType::PLUS: {
                            auto left = value_stack.back().value.value();
                            value_stack.pop_back();
                            auto right = t.value.value();
                            value_stack.push_back(EvaluateBinary(left, right, [](auto l, auto r){return r + l;}));
                            break;
                        }
                        default: abort();
                    }
                } else {
                    value_stack.push_back(t);
                }
                break;
            }
            case TokenType::PLUS:
                operator_stack.push_back(t);
                break;
            case TokenType::IDENTIFIER:
                operator_stack.push_back(t);
                break;
            default: abort();
        }
    }
    if(!value_stack.empty()) {
        auto result = value_stack.back();
        value_stack.pop_back();
        return result;
    }
#endif


void EvaluateTokens(const TokenList& tokens, const VariableMap& variables, StoredProgram& program)
{
    if(tokens.size() == 0) {
        // Should evaluate to nothing.
        return;
    }
    if(tokens[0].type == TokenType::INTEGER) {
        // Line number - a program line
        auto line_number = std::get<int32_t>(tokens[0].value.value());
        auto line = TokenList(tokens.cbegin() + 1, tokens.cend());
        program[line_number] = line;
        return;
    }
    if(tokens[0].type == TokenType::REMARK) {
        return;
    }

    std::vector<std::pair<TokenIterator, TokenIterator>> subsets;
    TokenIterator begin = tokens.begin();
    TokenIterator colon = tokens.begin();
    while(colon != tokens.end()) {
        if((*colon).type == TokenType::COLON) {
            if(begin != colon) {
                subsets.push_back({begin, colon});
            }
            begin = colon;
        }
    }
    if(begin != colon) {
        subsets.push_back({begin, colon});
    }

    while(!subsets.empty()) {
        auto [begin, end] = subsets.back();
        subsets.pop_back();
        if((*begin).type == TokenType::OPEN_PAREN) {
            int count = 1;
            auto close = begin + 1;
            while(close != end && count > 0) {
                if((*close).type == TokenType::OPEN_PAREN) {
                    count++;
                } else if((*close).type == TokenType::CLOSE_PAREN) {
                    count--;
                }
            }
            if(close != end) {
                subsets.push_back({close, end});
            }
            subsets.push_back({begin, close});
        } else {
            // ??
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

    }
    virtual ~ASTBinary() {}
};


ASTNodePtr Parse(TokenIterator begin, TokenIterator end)
{
    // KEYWORD_OR_OPERATOR MULTIPLY, PLUS
    // INTEGER, FLOAT, STRING_LITERAL

    std::vector<ASTNodePtr> node_stack;

    while(begin < end) {
        auto token = *begin;
        switch(token.type) {
            case TokenType::KEYWORD_OR_OPERATOR:
                switch(std::get<TokenType>(token.value.value())) {
                    case TokenType::PLUS: {
                        ASTNodePtr left = node_stack.back();
                        node_stack.pop_back();
                        ASTNodePtr right = Parse(begin + 1, end);
                        auto plus = std::shared_ptr<ASTNode>(new ASTBinary(left, right, [](auto l, auto r){return r + l;}));
                        node_stack.push_back(plus);
                        break;
                    }
                    case TokenType::MULTIPLY: {
                        ASTNodePtr left = node_stack.back();
                        node_stack.pop_back();
                        ASTNodePtr right = Parse(begin + 1, end);
                        auto plus = std::shared_ptr<ASTNode>(new ASTBinary(left, right, [](auto l, auto r){return r * l;}));
                        node_stack.push_back(plus);
                        break;
                    }
                    default: abort(); break;
                }
                break;
            case TokenType::INTEGER: {
                ASTNodePtr constant = std::make_shared<ASTConstant>(std::get<int32_t>(token.value.value()));
                node_stack.push_back(constant);
                begin++;
                break;
            }
            case TokenType::FLOAT: {
                ASTNodePtr constant = std::make_shared<ASTConstant>(std::get<double>(token.value.value()));
                node_stack.push_back(constant);
                begin++;
                break;
            }
            case TokenType::STRING_LITERAL: {
                ASTNodePtr constant = std::make_shared<ASTConstant>(std::get<std::string>(token.value.value()));
                node_stack.push_back(constant);
                begin++;
                break;
            }
            case TokenType::IDENTIFIER: {
                // XXX missing subscripts
                ASTNodePtr var = std::make_shared<ASTVariableInstance>(VariableReference(str_toupper(std::get<std::string>(token.value.value())), 0));
                node_stack.push_back(var);
                begin++;
                break;
            }
            default: abort(); break;
        }
    }

    return node_stack[0];
}
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

    StoredProgram program;

    VariableMap variables;
    variables["A"] = 123.0;
    variables["B"] = "Hello World!";

#if 0
    if(false) {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("B", 0));
        auto strun = ASTString1Param(node, [](auto v){return static_cast<int32_t>(v.size());});
        auto result = strun.evaluateR(variables, program);
        printf("%s\n", to_string(result).c_str());
    }

    if(false) {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("B", 0));
        auto len = std::make_shared<ASTConstant>(5);
        auto strun = ASTString2Param(node, len, [](auto v1, auto v2){return v1.substr(0, v2);});
        auto result = strun.evaluateR(variables, program);
        printf("%s\n", to_string(result).c_str());
    }

    if(false) {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("A", 0));
        auto un = ASTNumberUnary(node, [](auto v){return ! v;});
        auto result = un.evaluateR(variables, program);
        printf("%s\n", to_string(result).c_str());
    }

    if(true) {
        auto left = std::make_shared<ASTVariableInstance>(VariableReference("A", 0));
        auto right = std::make_shared<ASTConstant>(100);
        auto bin = std::shared_ptr<ASTNode>(new ASTBinary(left, right, [](auto l, auto r){return r - l;}));
        auto result = bin->evaluateR(variables, program);
        printf("%s\n", to_string(result).c_str());
    }
#endif

    if(true) {
        std::set<std::string> identifiers;
        while(fgets(line, sizeof(line), stdin) != NULL) {
            line[strlen(line) - 1] = '\0';
            try {
                auto tokens = Tokenize(line);
                print_tokenized(tokens);
                for(const auto& token: tokens) {
                    if(token.type == TokenType::IDENTIFIER) {
                        auto value = token.value.value();
                        identifiers.insert(std::get<std::string>(value).c_str());
                    }
                }
                EvaluateTokens(tokens, variables, program);
            } catch (const TokenizeError& e) {
                switch(e.type) {
                    case TokenizeError::SYNTAX:
                        printf("syntax error at %d (\"%5s\")\n", e.position, line + e.position);
                }
#if 0
            } catch (const InvalidLValueError& e) {
                printf("expected an l-value but none available\n");
            } catch (const TypeMismatchError& e) {
                printf("expected a number, encountered a string\n");
#endif
            } catch (const VariableNotFoundError& e) {
                printf("unknown variable \"%s\"\n", e.var.c_str());
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

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
parser - produce AST? for tokens
    AST parse(const Tokens& tokens); - ???
evaluate AST
    evaluate(AST, stack, variables, program)
    evaluate(AST, variables, program)
        evaluate(AST, stack(), variables, program)
    invoke functions
    store program

vector<vector<KeywordOrOperator>> PrecedenceTable =
{
    // what about NOT and unary MINUS?
    { POWER },
    { MULTIPLY, DIVIDE, MODULO, },
    { PLUS, MINUS, },
    { LESS_THAN, GREATER_THAN, LESS_THAN_EQUAL, MORE_THAN_EQUAL },
    { EQUALS, NOT_EQUAL},
    { AND, OR },
    { COMMA, SEMICOLON },
    { COLON, }, // statements
};

// require OPEN_PAREN, then expressions separated by COMMA, then CLOSE_PAREN
std::set<KeywordOrOperator> Functions =
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
    PRINT {;,,,expression} ... [;]
    INPUT [STRING_LITERAL SEMICOLON] expression list

Unknown
    identifier EQUAL expression

Leaves (no precedence)
    IDENTIFIER
    INTEGER
    FLOAT
    STRING_LITERAL
    Buuut, what about IDENTIFIER OPEN_PAREN list CLOSE_PAREN?

*/

enum class KeywordOrOperator
{
    NONE,
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

std::map<std::string, KeywordOrOperator> StringToKeywordOrOperatorMap =
{
    {"<>", KeywordOrOperator::NOT_EQUAL},
    {"<=", KeywordOrOperator::LESS_THAN_EQUAL},
    {">=", KeywordOrOperator::MORE_THAN_EQUAL},
    {">", KeywordOrOperator::MORE_THAN_EQUAL},
    {"<", KeywordOrOperator::LESS_THAN},
    {"(", KeywordOrOperator::OPEN_PAREN},
    {")", KeywordOrOperator::CLOSE_PAREN},
    {"=", KeywordOrOperator::EQUAL},
    {"+", KeywordOrOperator::PLUS},
    {"-", KeywordOrOperator::MINUS},
    {"*", KeywordOrOperator::MULTIPLY},
    {"/", KeywordOrOperator::DIVIDE},
    {"%", KeywordOrOperator::MODULO},
    {"^", KeywordOrOperator::POWER},
    {",", KeywordOrOperator::COMMA},
    {":", KeywordOrOperator::COLON},
    {";", KeywordOrOperator::SEMICOLON},
    {"DIM", KeywordOrOperator::DIM},
    {"LET", KeywordOrOperator::LET},
    {"IF", KeywordOrOperator::IF},
    {"THEN", KeywordOrOperator::THEN},
    {"ELSE", KeywordOrOperator::ELSE},
    {"FOR", KeywordOrOperator::FOR},
    {"TO", KeywordOrOperator::TO},
    {"STEP", KeywordOrOperator::STEP},
    {"NEXT", KeywordOrOperator::NEXT},
    {"GOTO", KeywordOrOperator::GOTO},
    {"GOSUB", KeywordOrOperator::GOSUB},
    {"RETURN", KeywordOrOperator::RETURN},
    {"PRINT", KeywordOrOperator::PRINT},
    {"INPUT", KeywordOrOperator::INPUT},
    {"END", KeywordOrOperator::END},
    {"ABS", KeywordOrOperator::ABS},
    {"ATN", KeywordOrOperator::ATN},
    {"COS", KeywordOrOperator::COS},
    {"EXP", KeywordOrOperator::EXP},
    {"INT", KeywordOrOperator::INT},
    {"LOG", KeywordOrOperator::LOG},
    {"RND", KeywordOrOperator::RND},
    {"SGN", KeywordOrOperator::SGN},
    {"SIN", KeywordOrOperator::SIN},
    {"SQR", KeywordOrOperator::SQR},
    {"TAN", KeywordOrOperator::TAN},
    {"AND", KeywordOrOperator::AND},
    {"OR", KeywordOrOperator::OR},
    {"NOT", KeywordOrOperator::NOT},
    {"LEFT$", KeywordOrOperator::LEFT},
    {"RIGHT$", KeywordOrOperator::RIGHT},
    {"MID$", KeywordOrOperator::MID},
    {"STR$", KeywordOrOperator::STR},
    {"LEN", KeywordOrOperator::LEN},
    {"TAB", KeywordOrOperator::TAB},
    {"WAIT", KeywordOrOperator::WAIT},
    {"DEF", KeywordOrOperator::DEF},
    {"FN", KeywordOrOperator::FN},
    {"CHR$", KeywordOrOperator::CHR},
    {"WIDTH", KeywordOrOperator::WIDTH},
    {"CLEAR", KeywordOrOperator::CLEAR},
    {"ON", KeywordOrOperator::ON},
    {"READ", KeywordOrOperator::READ},
    {"DATA", KeywordOrOperator::DATA},
};

std::unordered_map<KeywordOrOperator, const char *> KeywordOrOperatorToStringMap =
{
    {KeywordOrOperator::EQUAL, "="},
    {KeywordOrOperator::NOT_EQUAL, "<>"},
    {KeywordOrOperator::LESS_THAN_EQUAL, "<="},
    {KeywordOrOperator::MORE_THAN_EQUAL, ">="},
    {KeywordOrOperator::LESS_THAN, "<"},
    {KeywordOrOperator::MORE_THAN, ">"},
    {KeywordOrOperator::OPEN_PAREN, "("},
    {KeywordOrOperator::CLOSE_PAREN, ")"},
    {KeywordOrOperator::PLUS, "+"},
    {KeywordOrOperator::MINUS, "-"},
    {KeywordOrOperator::MULTIPLY, "*"},
    {KeywordOrOperator::DIVIDE, "/"},
    {KeywordOrOperator::MODULO, "%"},
    {KeywordOrOperator::POWER, "^"},
    {KeywordOrOperator::COMMA, ","},
    {KeywordOrOperator::COLON, ":"},
    {KeywordOrOperator::SEMICOLON, ";"},
    {KeywordOrOperator::DIM, "DIM"},
    {KeywordOrOperator::LET, "LET"},
    {KeywordOrOperator::IF, "IF"},
    {KeywordOrOperator::THEN, "THEN"},
    {KeywordOrOperator::ELSE, "ELSE"},
    {KeywordOrOperator::FOR, "FOR"},
    {KeywordOrOperator::TO, "TO"},
    {KeywordOrOperator::STEP, "STEP"},
    {KeywordOrOperator::NEXT, "NEXT"},
    {KeywordOrOperator::GOTO, "GOTO"},
    {KeywordOrOperator::GOSUB, "GOSUB"},
    {KeywordOrOperator::RETURN, "RETURN"},
    {KeywordOrOperator::PRINT, "PRINT"},
    {KeywordOrOperator::INPUT, "INPUT"},
    {KeywordOrOperator::END, "END"},
    {KeywordOrOperator::ABS, "ABS"},
    {KeywordOrOperator::ATN, "ATN"},
    {KeywordOrOperator::COS, "COS"},
    {KeywordOrOperator::EXP, "EXP"},
    {KeywordOrOperator::INT, "INT"},
    {KeywordOrOperator::LOG, "LOG"},
    {KeywordOrOperator::RND, "RND"},
    {KeywordOrOperator::SGN, "SGN"},
    {KeywordOrOperator::SIN, "SIN"},
    {KeywordOrOperator::SQR, "SQR"},
    {KeywordOrOperator::TAN, "TAN"},
    {KeywordOrOperator::AND, "AND"},
    {KeywordOrOperator::OR, "OR"},
    {KeywordOrOperator::NOT, "NOT"},
    {KeywordOrOperator::LEFT, "LEFT$"},
    {KeywordOrOperator::RIGHT, "RIGHT$"},
    {KeywordOrOperator::MID, "MID$"},
    {KeywordOrOperator::STR, "STR$"},
    {KeywordOrOperator::LEN, "LEN"},
    {KeywordOrOperator::TAB, "TAB"},
    {KeywordOrOperator::WAIT, "WAIT"},
    {KeywordOrOperator::DEF, "DEF"},
    {KeywordOrOperator::FN, "FN"},
    {KeywordOrOperator::CHR, "CHR$"},
    {KeywordOrOperator::WIDTH, "WIDTH"},
    {KeywordOrOperator::CLEAR, "CLEAR"},
    {KeywordOrOperator::ON, "ON"},
    {KeywordOrOperator::READ, "READ"},
    {KeywordOrOperator::DATA, "DATA"},
};

enum class TokenType
{
    KEYWORD_OR_OPERATOR,
    IDENTIFIER,
    INTEGER,
    FLOAT,
    STRING_LITERAL,
    REMARK,  		 // Remark (comment) starting with REM keyword
};

class Token
{
    public:
        TokenType type;
        std::optional<std::variant<std::string, int32_t, double, KeywordOrOperator>> value;

        Token(TokenType type) : type(type) {}
        Token(TokenType type, KeywordOrOperator kwop) : type(type), value(kwop) {}
        Token(TokenType type, const std::string& value) : type(type), value(value) {}
        Token(int32_t value) : type(TokenType::INTEGER), value(value) {}
        Token(double value) : type(TokenType::FLOAT), value(value) {}
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

std::vector<Token> Tokenize(const std::string& line)
{
    std::vector<Token> tokens;
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

        auto [matched, result] = [&](){
            for(const auto& [word, result]: StringToKeywordOrOperatorMap) {
                if(str_toupper(line.substr(index, word.size())) == word.c_str()) {
                    return std::make_tuple(word, result);
                }
            }
            return std::make_tuple(std::string(""), KeywordOrOperator::NONE);
        }();

        if (result != KeywordOrOperator::NONE)
        {
            flush_pending();
            tokens.push_back(Token(TokenType::KEYWORD_OR_OPERATOR, result));
            index += matched.size();
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

void print_tokenized(const std::vector<Token>& tokens)
{
    printf("%zd tokens: ", tokens.size());
    for(const auto& t: tokens) {
        auto v = t.value.value();
        switch(t.type) {
            case TokenType::KEYWORD_OR_OPERATOR: {
                auto ko = std::get<KeywordOrOperator>(v);
                printf("%s ", KeywordOrOperatorToStringMap[ko]);
                break;
            }
            case TokenType::IDENTIFIER: {
                printf("%s ", std::get<std::string>(v).c_str());
                break;
            }
            case TokenType::FLOAT:
                printf("%.f ", std::get<double>(v));
                break;
            case TokenType::INTEGER:
                printf("%d ", std::get<int32_t>(v));
                break;
            case TokenType::REMARK:
                printf("REM%s ", std::get<std::string>(v).c_str());
                break;
            case TokenType::STRING_LITERAL:
                printf("\"%s\" ", std::get<std::string>(v).c_str());
                break;
        }
    }
    printf("\n");
}

typedef std::variant<std::string, int32_t, double> Value;
typedef std::variant<Value, std::vector<Value>> MultiValue;
typedef std::map<std::string, MultiValue> VariableMap;
// Vector is indices into dimensioned variables
typedef std::pair<std::string, int> VariableReference;

// XXX Cases that need VariableReference: INPUT, READ, left side of EQUALS

Value Evaluate(const VariableReference& ref, const VariableMap& variables)
{
    auto iter = variables.find(ref.first);
    if(iter == variables.end()) {
        // throw a variable not found error
    }
    auto mvalue = iter->second;
    if(std::holds_alternative<std::vector<Value>>(mvalue)) {
        return std::get<std::vector<Value>>(mvalue)[ref.second];
    }
    return std::get<Value>(mvalue);
}

struct InvalidLValueError { };
struct TypeMismatchError { };

struct ASTNode
{
    virtual Value evaluateR(VariableMap& variables) = 0;
    virtual VariableReference evaluateL(VariableMap& variables) {
        throw InvalidLValueError();
    }
    virtual ~ASTNode() {}
};

typedef std::shared_ptr<ASTNode> ASTNodePtr;

template <class Op>
struct ASTNumberUnary : public ASTNode
{
    ASTNodePtr node;
    Op op;
    ASTNumberUnary(ASTNodePtr node, Op op) :
        node(std::move(node)),
        op(op)
    {}
    virtual Value evaluateR(VariableMap& variables) override
    {
        Value v = node->evaluateR(variables);
        if(std::holds_alternative<std::string>(v)) {
            throw TypeMismatchError();
        }
        if(std::holds_alternative<double>(v)) {
            return op(std::get<double>(v));
        } else {
            return op(std::get<int32_t>(v));
        }
    }
    virtual ~ASTNumberUnary() {}
};

template <class Op>
struct ASTString1Param : public ASTNode
{
    ASTNodePtr node;
    Op op;
    ASTString1Param(ASTNodePtr node, Op op) :
        node(std::move(node)),
        op(op)
    {}
    virtual Value evaluateR(VariableMap& variables) override
    {
        Value v = node->evaluateR(variables);
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
    virtual Value evaluateR(VariableMap& variables) override
    {
        Value value1 = param1->evaluateR(variables);
        Value value2 = param2->evaluateR(variables);
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

template <class Op>
struct ASTBinary : public ASTNode
{
    ASTNodePtr left;
    ASTNodePtr right;
    Op op;
    ASTBinary(ASTNodePtr left, ASTNodePtr right, Op op) :
        left(std::move(left)),
        right(std::move(right)),
        op(op)
    {}
    virtual Value evaluateR(VariableMap& variables) override
    {
        Value r = right->evaluateR(variables);
        Value l = left->evaluateR(variables);
        if(std::holds_alternative<std::string>(r)) {
            throw TypeMismatchError();
        }
        if(std::holds_alternative<std::string>(l)) {
            throw TypeMismatchError();
        }
        if(std::holds_alternative<double>(l) && std::holds_alternative<double>(r)) {
            return op(std::get<double>(l), std::get<double>(r));
        } else if(std::holds_alternative<double>(l)) {
            return op(std::get<double>(l), std::get<int32_t>(r));
        } else if(std::holds_alternative<double>(r)) {
            return op(std::get<int32_t>(l), std::get<double>(r));
        } else {
            return op(std::get<int32_t>(l), std::get<int32_t>(r));
        }
    }
    virtual ~ASTBinary() {}
};

struct ASTVariableInstance : public ASTNode
{
    VariableReference ref;
    ASTVariableInstance(const VariableReference& ref) :
        ref(ref)
    {}
    virtual VariableReference evaluateL(VariableMap& variables) override {
        return ref;
    }
    virtual Value evaluateR(VariableMap& variables) override {
        return Evaluate(ref, variables);
    }
    virtual ~ASTVariableInstance() {}
};

struct ASTConstant : public ASTNode
{
    Value val;
    ASTConstant(const Value& val) :
        val(val)
    {}
    virtual Value evaluateR(VariableMap& variables) override {
        return val;
    }
    virtual ~ASTConstant() {}
};

ASTNodePtr Parse(const std::vector<Token>& tokens)
{
}

std::string to_string(const Value& v)
{
    if(std::holds_alternative<int32_t>(v)) {
        return std::to_string(std::get<int32_t>(v));
    } else if(std::holds_alternative<double>(v)) {
        return std::to_string(std::get<double>(v));
    } else if(std::holds_alternative<std::string>(v)) {
        return std::get<std::string>(v).c_str();
    } else return "unknown"; // XXX variant should handle this for me.
}

int main(int argc, char **argv)
{
    char line[1024];
    std::set<std::string> identifiers;

    VariableMap variables;
    variables["A"] = 123.0;
    variables["B"] = "Hello World!";

    {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("B", 0));
        auto strun = ASTString1Param(node, [](auto v){return static_cast<int32_t>(v.size());});
        auto result = strun.evaluateR(variables);
        printf("%s\n", to_string(result).c_str());
    }

    {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("B", 0));
        auto len = std::make_shared<ASTConstant>(5);
        auto strun = ASTString2Param(node, len, [](auto v1, auto v2){return v1.substr(0, v2);});
        auto result = strun.evaluateR(variables);
        printf("%s\n", to_string(result).c_str());
    }

    {
        auto node = std::make_shared<ASTVariableInstance>(VariableReference("A", 0));
        auto un = ASTNumberUnary(node, [](auto v){return ! v;});
        auto result = un.evaluateR(variables);
        printf("%s\n", to_string(result).c_str());
    }

    {
        auto left = std::make_shared<ASTVariableInstance>(VariableReference("A", 0));
        auto right = std::make_shared<ASTConstant>(100);
        auto bin = ASTBinary(left, right, [](auto l, auto r){return r - l;});
        auto result = bin.evaluateR(variables);
        printf("%s\n", to_string(result).c_str());
    }

    if(false) {
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
            } catch (const TokenizeError& e) {
                switch(e.type) {
                    case TokenizeError::SYNTAX:
                        printf("syntax error at %d (\"%5s\")\n", e.position, line + e.position);
                }
            } catch (const InvalidLValueError& e) {
                printf("expected an l-value but none available\n");
            } catch (const TypeMismatchError& e) {
                printf("expected a number, encountered a string\n");
            }
        }
        printf("identifiers:\n");
        for(const auto &id: identifiers) {
            printf("%s\n", id.c_str());
        }
    }
}

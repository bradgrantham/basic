#include <cstdio>
#include <cctype>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <optional>
#include <unordered_map>

std::unordered_map<std::string, int> operator_precedence = {
    { ")", -999 }, 
    { "(", -1000 }, 
    { "SIN", -1000 }, 
    { "COS", -1000 }, 
    { "TAN", -1000 }, 
    { "SGN", -1000 }, 
    { "RND", -1000 }, 
    { "LOG", -1000 }, 
    { "INT", -1000 }, 
    { "EXP", -1000 }, 
    { "^", 6 }, 
    { "*", 5 }, 
    { "/", 5 }, 
    { "+", 4 }, 
    { "-", 4 }, 
    { "<", 3 }, 
    { ">", 3 }, 
    { "<=", 3 }, 
    { ">=", 3 }, 
    { "=", 2 }, 
    { "<>", 2 }, 
    { "AND", 2 }, 
    { "OR", 2 }, 
};

std::set<std::string> operator_strings = {
    "(", ")", "^", "*", "/", "+", "-", "<", ">", ">=", "<=", "=", "<>", "AND", "OR", "NOT"
};

std::set<std::string> binary_operators = {
    "^", "*", "/", "+", "-", "<", ">", ">=", "<=", "=", "<>", "AND", "OR"
};

std::set<std::string> unary_operators = {
    "+", "-", "NOT"
};

std::set<std::string> commands = {
    "LET", "PRINT", "DIM", "IF", "FOR", "NEXT", "ON", "GOTO", "GOSUB", "RETURN", "INPUT", "END", "WAIT", "DEF", "WIDTH", "CLEAR", "ORDER", "READ", "DATA"
};


std::tuple<float, float> pop2(std::vector<float>& operands)
{
    float right = operands.back(); operands.pop_back();
    float left = operands.back(); operands.pop_back();
    return {left, right};
}

template <typename Q>
auto pop(Q& queue)
{
    auto back = queue.back();
    queue.pop_back();
    return back;
}

void dump_operators(const std::vector<std::string>& operators)
{
    printf("operators: ");
    for(auto op: operators) { printf("\"%s\" ", op.c_str()); }
    printf("\n");
}

void dump_operands(const std::vector<float>& operands)
{
    printf("operands: ");
    for(auto op: operands) { printf("%f ", op); }
    printf("\n");
}

float evaluate(const std::string& op, std::vector<float>& operands)
{
    float right = pop(operands);
    if(op == ")") {
        return right;
    } else if(op == "(") {
        return right;
    } else if(op == "^") {
        float left = pop(operands);
        return pow(left,right);
    } else if(op == "*") {
        float left = pop(operands);
        return left * right;
    } else if(op == "/") {
        float left = pop(operands);
        return left / right;
    } else if(op == "+") {
        float left = pop(operands);
        return left + right;
    } else if(op == "-") {
        float left = pop(operands);
        return left - right;
    } else if(op == "SIN") {
        return sin(right);
    } else if(op == "COS") {
        return cos(right);
    } else if(op == "TAN") {
        return tan(right);
    } else if(op == "SGN") {
        return right < 0 ? -1 : (right > 0 ? 1 : 0);
    } else if(op == "INT") {
        return trunc(right);
    } else if(op == "RND") {
        return drand48();
    } else if(op == "EXP") {
        return exp(right);
    } else if(op == "LOG") {
        return log(right);
    } else {
        printf("internal error\n");
        abort();
    }
}

std::string str_toupper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::toupper(c); }
                  );
    return s;
}

std::optional<std::string> is_operator(const char *line, int *used)
{
    for(int i = 1; i < 4; i++) {
        std::string op;
        op.append(line, std::min(static_cast<size_t>(i), strlen(line)));
        op = str_toupper(op);
        if(operator_strings.count(op) > 0) {
            *used = i;
            return op;
        }
    }
    return std::nullopt;
}

bool is_binary_operator(const std::string op)
{
    return binary_operators.count(op) > 0;
}

bool is_unary_operator(const std::string op)
{
    return unary_operators.count(op) > 0;
}

bool is_higher_precedence(const std::string& op1, const std::string& op2)
{
    if(op2 == ")") {
        return true;
    }
    if(op1 == "(") {
        return false;
    }
    bool both_binary = is_binary_operator(op1) && is_binary_operator(op2);
    bool is_higher = both_binary && (operator_precedence.at(op1) > operator_precedence.at(op2));
    return is_higher;
}

void evaluate_line(const char *line)
{
    int cur = 0;
    std::vector<float> operands;
    std::vector<float> results;
    std::vector<std::string> operators;
    std::vector<std::string> unary_operators;

    bool next_operator_is_unary = true;
    static char word[512];
    int used = 0;

    if(sscanf(line + cur, " %[A-Za-z]%n", word, &used) != 1) {
        printf("expected command to begin line\n");
    }
    std::string command{word};
    std::string variable;
    command = str_toupper(command);
    // if(commands.count(command) == 0) {
        // variable = command;
        // command = "LET";
    // }
    cur += used;

    while(line[cur]) {
        float f = -666;

        while(line[cur] && isspace(line[cur])) {
            cur++;
        }

        if(line[cur] == ',') {
            assert(operands.size() == 1);
            results.push_back(pop(operands));
        } else if(line[cur] == '(') {
            operators.push_back("(");
            used = 1;
            next_operator_is_unary = true;
        } else if(auto result = is_operator(line + cur, &used)) {
            auto op = *result;
            if(next_operator_is_unary) {
                if(is_unary_operator(op)) {
                    unary_operators.push_back(op);
                } else {
                    printf("unexpected operator in unary context: \"%s\"\n", op.c_str());
                    abort();
                }
            } else {
                while(!operators.empty() && is_higher_precedence(operators.back(), op)) {
                    std::string higher = pop(operators);
                    operands.push_back(evaluate(higher, operands));
                }
                operators.push_back(op);
                next_operator_is_unary = is_binary_operator(op);
            }
        } else if(sscanf(line + cur, "%f%n", &f, &used) == 1) {
            while(!unary_operators.empty()) {
                std::string op = pop(unary_operators);
                if(op == "-") {
                    f = -f;
                } else if(op == "NOT") {
                    f = -1 - f;
                } else if(op == "+") {
                    // f = f;
                } else {
                    printf("internal error, unary operator \"%s\"\n", op.c_str());
                }
            }
            operands.push_back(f);
            next_operator_is_unary = false;
        } else if(sscanf(line + cur, "%[A-Za-z]%n", word, &used) == 1) {
            std::string func{str_toupper(word)};
            if(operator_precedence.count(func) == 0) {
                printf("unknown reserved word \"%s\"\n", func.c_str());
                abort();
            }
            operators.push_back(func);
            next_operator_is_unary = true;
        } else {
            printf("syntax error at \"%s\"\n", line + cur);
            abort();
        }
        assert(used != 0);
        cur += used;
    }
    while(!operators.empty()) {
        std::string op = pop(operators);
        operands.push_back(evaluate(op, operands));
    }
#if 0
    if(operands.size() > 0) {
        printf("%f\n", operands.back());
    }
    if(operands.size() > 1) {
        printf("extra ");
        dump_operands(operands);
    }
#else
    if(operands.size() > 0) {
        results.push_back(operands.back());
    }
    if(command == "PRINT") {
        for(auto f: results) {
            printf("%f ", f);
        }
        printf("\n");
    }
#endif
}

int main(int argc, char **argv)
{
    static char line[512];
    while(fgets(line, sizeof(line), stdin) != nullptr) {
        line[strlen(line) - 1] = '\0';

        evaluate_line(line);
    }
}

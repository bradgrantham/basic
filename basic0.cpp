#include <cstdio>
#include <cctype>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <optional>
#include <unordered_map>
#include <map>

const bool debug_state = false;

std::unordered_map<std::string, int> operator_precedence = {
    { "SIN", -1000 }, 
    { "COS", -1000 }, 
    { "TAN", -1000 }, 
    { "SGN", -1000 }, 
    { "RND", -1000 }, 
    { "LOG", -1000 }, 
    { "INT", -1000 }, 
    { "EXP", -1000 }, 
    { "TAB", -1000 }, 
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

std::set<std::string> function_strings = {
    "SIN",
    "COS",
    "TAN",
    "SGN",
    "RND",
    "LOG",
    "INT",
    "EXP",
    "TAB",
};

std::set<std::string> operator_strings = {
    "^", "*", "/", "+", "-", "<", ">", ">=", "<=", "=", "<>", "AND", "OR", "NOT"
};

std::set<std::string> binary_operators = {
    "^", "*", "/", "+", "-", "<", ">", ">=", "<=", "=", "<>", "AND", "OR"
};

std::set<std::string> unary_operators = {
    "+", "-", "NOT"
};

std::set<std::string> commands = {
    "RUN", "LET", "PRINT", "DIM", "IF", "FOR", "NEXT", "ON", "GOTO", "GOSUB", "RETURN", "INPUT", "END", "WAIT", "DEF", "WIDTH", "CLEAR", "ORDER", "READ", "DATA"
};

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
std::string str(const Value& v) { return std::get<std::string>(v); }
double num(const Value& v) { return std::get<double>(v); }
VariableReference vref(const Value& v) { return std::get<VariableReference>(v); }
bool is_vref(const Value& v) { return std::holds_alternative<VariableReference>(v); }
bool is_str(const Value& v) { return std::holds_alternative<std::string>(v); }
bool is_num(const Value& v) { return std::holds_alternative<double>(v); }

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

Value evaluate(const std::string& op, std::vector<Value>& operands)
{
    Value right = pop(operands);
    if(op == ")") {
        return right;
    } else if(op == "(") {
        return right;
    } else if(op == "^") {
        Value left = num(pop(operands));
        return pow(num(left), num(right));
    } else if(op == "*") {
        Value left = pop(operands);
        return num(left) * num(right);
    } else if(op == "/") {
        Value left = pop(operands);
        return num(left) / num(right);
    } else if(op == "+") {
        Value left = pop(operands);
        return num(left) + num(right);
    } else if(op == "-") {
        Value left = pop(operands);
        return num(left) - num(right);
    } else if(op == "TAB") {
        return std::string(static_cast<int>(num(right)), ' ');
    } else if(op == "SIN") {
        return sin(num(right));
    } else if(op == "COS") {
        return cos(num(right));
    } else if(op == "TAN") {
        return tan(num(right));
    } else if(op == "SGN") {
        return num(right) < 0.0 ? -1.0 : (num(right) > 0.0 ? 1.0 : 0.0);
    } else if(op == "INT") {
        return trunc(num(right));
    } else if(op == "RND") {
        return drand48();
    } else if(op == "EXP") {
        return exp(num(right));
    } else if(op == "LOG") {
        return log(num(right));
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

bool is_binary_operator(const std::string& op)
{
    return binary_operators.count(op) > 0;
}

bool is_unary_operator(const std::string& op)
{
    return unary_operators.count(op) > 0;
}

bool is_function(const std::string &word)
{
    return function_strings.count(word) > 0;
}

bool is_command(const std::string &command)
{
    return commands.count(command) > 0;
}

bool is_higher_precedence(const std::string& op1, const std::string& op2)
{
    bool both_binary = is_binary_operator(op1) && is_binary_operator(op2);
    bool is_higher = both_binary && (operator_precedence.at(op1) > operator_precedence.at(op2));
    return is_higher;
}

struct State
{
    std::unordered_map<std::string, Value> variables;
    std::map<int, std::string> program;
    int current_line{-1};
    int goto_line{-1};
    bool direct{true};
};

void evaluate_line(const char *line, State& state)
{
    int cur = 0;
    std::vector<Value> operands;
    std::vector<std::string> operators;
    std::vector<std::string> unary_operators;

    auto skip_whitespace = [&]() {
        while(line[cur] && isspace(line[cur])) {
            cur++;
        }
    };


    bool next_operator_is_unary = true;
    char word[512];
    int used = 0;

    skip_whitespace();
    int line_number;
    if(sscanf(line, " %d %n", &line_number, &used) == 1) {
        state.program[line_number] = line + used;
        // printf("%d \"%s\"\n", line_number, state.program.at(line_number).c_str());
        return;
    }

    if(sscanf(line + cur, " %[A-Za-z] %n", word, &used) != 1) {
        printf("expected command to begin line\n");
    }
    cur += used;
    std::string command{word};
    command = str_toupper(command);

    if(!is_command(command) && !is_function(command)) {
        skip_whitespace();
        if(line[cur] != '=')
        // if(sscanf(line + cur, "=%n", &used) != 1)
        {
            printf("expected \"=\" at %d, \"%s\"\n", cur, line + cur);
            abort();
        }
        used = 1;
        operands.push_back(VariableReference(str_toupper(command), {}));
        cur += used;
        command = "LET";
    } else if(command == "LET") {
        // XXX quick hack before making variable references
        skip_whitespace();
        if(sscanf(line + cur, "%[A-Za-z] = %n", word, &used) != 1) {
            printf("expected variable name and \"=\" at %d, \"%s\"\n", cur, line + cur);
            abort();
        }
        operands.push_back(VariableReference(str_toupper(word), {}));
        cur += used;
    }

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
#else
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

        evaluate_line(line, state);
    }
}

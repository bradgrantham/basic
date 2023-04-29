#include <cstdio>
#include <cctype>
#include <string>
#include <vector>
#include <algorithm>
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
};
    // { LESS_THAN, GREATER_THAN, LESS_THAN_EQUAL, GREATER_THAN_EQUAL },
    // { EQUAL, NOT_EQUAL},
    // { AND, OR },

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

int main(int argc, char **argv)
{
    static char line[512];
    while(fgets(line, sizeof(line), stdin) != nullptr) {
        line[strlen(line) - 1] = '\0';

        int cur = 0;
        std::vector<float> operands;
        std::vector<std::string> operators;

        while(line[cur]) {
            float f = -666;
            int used = 0;
            static char word[512];
            while(line[cur] && isspace(line[cur])) {
                cur++;
            }
            if(sscanf(line + cur, " %n", &used) == 1) {
                // pass; whitespace
            } else if(sscanf(line + cur, "%[()-+/*^]%n", word, &used) == 1) {
                std::string op;
                op.append(word, 1);
                while((!operators.empty()) && (operator_precedence.at(operators.back()) > operator_precedence.at(op))) {
                    std::string higher = pop(operators);
                    operands.push_back(evaluate(higher, operands));
                }
                operators.push_back(op);
                used = 1;
            } else if(sscanf(line + cur, "%f%n", &f, &used) == 1) {
                operands.push_back(f);
            } else if(sscanf(line + cur, "%[A-Za-z]%n", word, &used) == 1) {
                std::string func{str_toupper(word)};
                if(operator_precedence.count(func) == 0) {
                    printf("unknown reserved word \"%s\"\n", func.c_str());
                    abort();
                }
                operators.push_back(func);
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
        if(operands.size() > 0) {
            printf("%f\n", operands.back());
        }
        if(operands.size() > 1) {
            printf("extra ");
            dump_operands(operands);
        }
    }
}

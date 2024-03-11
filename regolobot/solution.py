import copy


class Solution:
    def __init__(self, math_sticks):
        # A simple class representing a solution for the regolobot
        # problem. I decided to use a class since it is much easier
        # and less computationally expensive to keep track of the value
        # if elements are added one at a time.
        # Apparent solution format: ["stick_pos", "op", "stick_pos", "op"...]
        # Underlying solution format: [(...), (...)] where each element
        # has only multiplications inside.
        self._hierarchy = []
        self.mathsticks = math_sticks
        self._ops = ["+", "*"]
        self._last_is_op = False
        self._value = None
        self._updated = False

    def _already_in_hierarchy(self, position: int):
        for addend in self._hierarchy:
            for factor in addend:
                if position in addend:
                    return True
        return False

    def add_stick(self, position: int):
        if position > len(self.mathsticks) - 1:
            raise ValueError("Error building solution: stick does not exist")
        if len(self._hierarchy) == 0:
            self._hierarchy.append([position])
        elif not self._already_in_hierarchy(position):
            self._hierarchy[-1].append(position)
        else:
            raise ValueError("Error building solution: stick already in use")
        self._last_is_op = False
        self._updated = False

    def add_operator(self, op: str):
        if len(self._hierarchy) == 0:
            raise ValueError("Error building solution: missing operand")
        if op not in self._ops:
            raise ValueError("Error building solution: invalid operator")
        if op == "+":
            self._hierarchy.append([])
        self._last_is_op = True

    def remove(self):
        if not self._last_is_op:
            self._hierarchy[-1].pop()
            self._last_is_op = True
            self._updated = False
        else:
            if len(self._hierarchy[-1]) == 0:
                self._hierarchy.pop()
            self._last_is_op = False

    def clear(self):
        self._hierarchy = []

    def simplify(self):
        if self._last_is_op and len(self._hierarchy[-1]) == 0:
            self._hierarchy.pop()
        self._last_is_op = False

    @property
    def value(self):
        if self._value is not None and self.updated:
            return self._value
        total = 0
        for addend in self._hierarchy:
            if len(addend) == 0:
                break
            factor_total = self.mathsticks[addend[0]]
            for factor in addend[1:]:
                factor_total *= self.mathsticks[factor]
            total += factor_total
        self._updated = True
        return total

    @property
    def solution(self):
        solution = []
        len_last = 0
        for addend in self._hierarchy:
            if len(addend) == 0:
                len_last = 0
                break
            solution.append(addend[0])
            for factor in addend[1:]:
                solution.append("*")
                solution.append(factor)
            solution.append("+")
            len_last = len(addend)
        if len_last > 0:
            solution.pop()
        return solution

    @property
    def last_op(self):
        if len(self._hierarchy[-1]) == 0:
            return "+"
        return "*"

    def __mul__(self, other):
        if isinstance(other, int):
            self.add_stick(other)
            return self
        raise ValueError("Error building solution: invalid operand")

    def __add__(self, other):
        if isinstance(other, int):
            self.add_operator("+")
            self.add_stick(other)
            return self
        raise ValueError("Error building solution: invalid operand")

    def __len__(self):
        """Number of sticks in the current solution

        Returns:
            int: Number of sticks in the current solution
        """
        total = 0
        for addend in self._hierarchy:
            for factor in addend:
                total += 1
        return total

    def copy(self):
        new_solution = Solution(self.mathsticks)
        new_solution._hierarchy = copy.deepcopy(self._hierarchy)
        new_solution._last_is_op = self._last_is_op
        return new_solution


def backtrack_solution(
    math_sticks: list,
    target: int,
    solution: Solution = None,
    best_solution: Solution = None,
    offset=0,
) -> Solution:
    if len(math_sticks) == 0:
        return best_solution

    if solution is None:
        solution = Solution(math_sticks)

    if solution.value > target:
        return best_solution

    if solution.value == target and (
        best_solution is None or len(solution) < len(best_solution)
    ):
        best_solution = solution.copy()
        best_solution.simplify()
        return best_solution

    if best_solution is not None and best_solution.value == target:
        # If we found a solution having a correct value, then it is
        # automatically the best solution, since the math_sticks list
        # is sorted and we start with products.
        return best_solution.copy()

    # Prune if current solution + or - a stick is > target
    # Avoid multiplication with 1s
    for i, stick in enumerate(math_sticks):
        if stick > target:
            continue
        if stick == 1 and solution.last_op == "*":
            continue
        solution.add_stick(i + offset)
        solution.add_operator("*")
        best_solution = backtrack_solution(
            math_sticks[i + 1 :],
            target,
            solution,
            best_solution,
            offset=i + 1 + offset,
        )
        if best_solution is not None and best_solution.value == target:
            break
        solution.remove()
        solution.add_operator("+")
        best_solution = backtrack_solution(
            math_sticks[i + 1 :], target, solution, best_solution, offset=i + 1 + offset
        )
        if best_solution is not None and best_solution.value == target:
            break
        solution.remove()
        solution.remove()
    return best_solution


if __name__ == "__main__":

    math_sticks = [10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 5, 5, 4, 3, 3, 1, 1, 1]
    exit = False
    while not exit:
        target = input('Insert target number, or "exit" to exit: ')
        if target == "exit":
            break
        else:
            target = int(target)
        s = backtrack_solution(math_sticks, target)
        if s is None:
            print("No solution found")
        else:
            print(s._hierarchy)
            print(s.solution)
            print(s.value)

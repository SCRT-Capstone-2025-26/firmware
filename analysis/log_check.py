import re

class LogExpectation():
    def __init__(self, message, core=None, latest_time=None):
        self.latest_time = latest_time
        self.core = core
        self.message = message


    def matches(self, log):
        if not re.match(self.message, log.message):
            return False

        if self.core is not None and self.core != log.core:
            return False

        if self.latest_time is not None and self.latest_time < log.time:
            return False

        return True


    def __str__(self):
        return f'LogExpectation(message={self.message}, core={self.core}, latest_time={self.latest_time})'


class LogChecker():
    def __init__(self):
        self.expected = set()
        self.parents = {}


    def add_expected(self, expectation):
        self.expected.add(expectation)


    # Adding follows also adds them as expectations
    def add_follows(self, previous, expectation):
        self.expected.add(previous)
        self.expected.add(expectation)

        self.parents.setdefault(expectation, []).append(previous)


    def check(self, logs):
        seen = set()

        for log in logs:
            expectated = False
            # All matches are detected so all possible parents are detected
            for expectation in self.expected:
                if expectation.matches(log):
                    expectated = True
                    seen.add(expectation)

                    for parent in self.parents.get(expectation, []):
                        if parent not in seen:
                            return False, f'{parent} not before {expectation}'

            if not expectated:
                return False, f'No matches for {log}'

        for expectation in self.expected:
            if expectation not in seen:
                return False, f'Expected {expectation}, but not did not find a match'

        return True, ''


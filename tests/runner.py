import os
import subprocess

def getTestList():
	tests = []
	for file in os.listdir("./bin"):
		if ".bin" in file:
			tests.append("./bin/" + file)
	return tests

testNames = getTestList()
testsPassed = 0
testsRan = 0

def runTest(file):
	return subprocess.call([file, str(testsRan), str(len(testNames))])

print("\n")

for file in testNames:
	print(file)
	errored = runTest(file)

	if not errored:
		testsPassed += 1

	testsRan += 1

status = "\033[0;32m"

# check to see if any failed
if testsPassed != len(testNames):
	status = "\033[1;31m"

print(str(testsRan) + " of " + str(len(testNames)) + " ran")
print(status + str(testsPassed) + " of " + str(len(testNames)) + " passed\033[0m")

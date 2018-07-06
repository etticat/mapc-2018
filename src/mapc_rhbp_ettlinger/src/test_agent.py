pureResponseNames = ['a/f'+str(i) for i in range(14340)]
outpathDirTest="testCase"
output_files = [fname + ".txt" for fname in pureResponseNames]

for filename in output_files:
    with open(filename, 'w') as f:
        f.write('This is a test of file nr.'+str(i))
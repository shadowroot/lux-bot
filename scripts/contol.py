from simple_rpc import Interface
device = 'COM8'

method_keys = {}

print('Initialising... ')
interface = Interface(device)
print('done.\n\n')

print('Protocol version: {}\n\n'.format(interface.__dict__))

print('Available methods (end with 0): ')
for i, method_name in enumerate(interface.methods.keys(), start=1):
    method_keys[i] = method_name
    method = interface.methods[method_name]
    print('\t{}\t{}\t{}\t{}'.format(i, method.name, method.doc, method.parameters))

while method_number != None:
    method_number = input("Enter method number: ")
    if int(method_number) == 0:
        break
    if int(method_number) in method_keys:
        method = interface.methods[method_keys[int(method_number)]]
        args = []
        for parameter in method.parameters:
            arg = input("Enter method arguments: ")
            if parameter.typename == 'int':
                arg = int(arg)
            elif parameter.typename == 'float':
                arg = float(arg)
            else:
                raise Exception("Unknown type {}".format(parameter.typename))
            args.append(arg)
    print("Calling method {} with arguments {}".format(method.name, args))
    interface[method](*args)
print("End of sequence")
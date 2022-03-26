import json

string_example='''
{
    "people":[
        {
            "name":"John Smith",
            "phone":"615-555-716",
            "emails":["johnsmith@gamil.com", "john.smith@workemail.com"],
            "has_licence":false
        },
        {
            "name":"Jane Doe",
            "phone":"675-584-426",
            "emails":null,
            "has_licence":true
        }
    ]
}
'''

data = json.loads(string_example)

print(type(data))

for person in data['people']:
    print(person['name'])

# create json from string:
new_string = json.dumps(data,indent=2)

print(new_string)

with open('./wlasne programy/json_example_1.json', 'w') as f:
    json.dump(data,f,indent=2)

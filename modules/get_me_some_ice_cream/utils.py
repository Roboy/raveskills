def insert_user_name(text, username):
    return text.replace('$$', username)


def extract_scoops(prop_ner):
    for entity, NE in prop_ner:
        if NE == "CARDINAL" and isinstance(entity, int):
            return entity
        elif NE == "CARDINAL" and isinstance(entity, str):
            # we assume that no one orders more than 9 scoops of a flavor
            word2num = {
                "one": 1,
                "two": 2,
                "three": 3,
                "four": 4,
                "five": 5,
                "six": 6,
                "seven": 7,
                "eight": 8,
                "nine": 9,
            }
            return word2num[entity]

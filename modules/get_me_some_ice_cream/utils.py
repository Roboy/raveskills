def insert_user_name(text, username):
    return text.replace('$$', username)


def extract_scoops(prop_ner):
    for entity, NE in prop_ner:
        if NE == "CARDINAL" and entity.isdigit():
            return int(entity)
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


def get_complete_order_and_cost(flavor_scoop_tuple_list, cost_per_scoop):
    order = ""
    cost = 0
    if len(flavor_scoop_tuple_list) == 1:
        order = "{scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[0][0],
                                                     scoops=flavor_scoop_tuple_list[0][1])
        cost = cost_per_scoop * flavor_scoop_tuple_list[0][1]
    else:
        order_length = len(flavor_scoop_tuple_list)
        for i in range(0, order_length - 1):
            order += "{scoops} scoops of {flavor}, ".format(flavor=flavor_scoop_tuple_list[i][0],
                                                                   scoops=flavor_scoop_tuple_list[i][1])
            cost += cost_per_scoop * flavor_scoop_tuple_list[i][1]
        order = order[:len(order)-2] + " "
        order += "and {scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[order_length-1][0],
                                                                   scoops=flavor_scoop_tuple_list[order_length-1][1])
        cost += cost_per_scoop * flavor_scoop_tuple_list[order_length-1][1]
    return order, cost


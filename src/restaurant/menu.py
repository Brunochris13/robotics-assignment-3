from easydict import EasyDict as edict

'''
value out : list
- list[0]: price
- list[1]: only available to 18+
'''

menu = edict()

menu.pizza = edict()
menu.pizza.Margherita = (5.55, False)
menu.pizza.Hawaiian =(5.95, False)
menu.pizza.Italian = (6.35, False)
menu.pizza.vegan_Margherita = (6.75, False)
menu.pizza.Chicken_Tikka = (6.75, False)
menu.pizza.Vegetarian = (6.35, False)
menu.pizza.Chicken_Supreme = (6.35, False)
menu.pizza.BBQ_Meat_Feast = (6.85, False)
menu.pizza.Goatee = (6.85, False)

menu.Burger = edict()
menu.Burger.Classic = (5.95, False)
menu.Burger.Cheese_N_Bacon = (6.95, False)
menu.Burger.Vegetable_Burger = (5.5, False)

menu.drink = edict()
menu.drink.Coca_cola = (2., False)
menu.drink.Sprite = (2., False)
menu.drink.Sneak = (2., False)
menu.drink.Carabao = (2., False)
menu.drink.Red_Bull = (3., False)
menu.drink.Dragon = (1.5, False)
menu.drink.Rubicon_Raw = (1.5, False)
menu.drink.Warrior = (1.3, False)
menu.drink.Monster = (2., False)
menu.drink.Tenzing = (1., False)
menu.drink.Corona = (2., True)

menu.robotics_special_menu = edict()
menu.robotics_special_menu.Espresso_20shots = (20., False)
menu.robotics_special_menu.Red_Bull_5cans = (15., False)
menu.robotics_special_menu.Caffeine_6pills = (12., False)
menu.robotics_special_menu.Lemsip = (4.69, False)
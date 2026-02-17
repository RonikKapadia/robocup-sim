import pygame
from source.ENVIRONMENT import RoboCupEnv


if __name__ == '__main__':
    
    # create gym environment
    env = RoboCupEnv()

    # initialize gym environment
    obs = env.reset()
    env.render_init(show_debug=True)

    # run game loop
    run = True
    while run:
        
        # get/handle user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            keys = pygame.key.get_pressed()

            action = [0.5,0.5,0,0.5] # [dx, dy, v1, a1]
            if keys[pygame.K_a]: # move left
                action[0] = (-1 + 1)/2
                action[1] = ( 0 + 1)/2
                action[2] = 1.0
            if keys[pygame.K_d]: # move right
                action[0] = ( 1 + 1)/2
                action[1] = ( 0 + 1)/2
                action[2] = 1.0
            if keys[pygame.K_w]: # move up
                action[0] = ( 0 + 1)/2
                action[1] = (-1 + 1)/2
                action[2] = 1.0
            if keys[pygame.K_s]: # move down
                action[0] = ( 0 + 1)/2
                action[1] = ( 1 + 1)/2
                action[2] = 1.0
            if keys[pygame.K_q]: # rotate anti-clockwise
                action[3] = (-1 + 1)/2
            if keys[pygame.K_e]: # rotate clockwise
                action[3] = ( 1 + 1)/2
            if keys[pygame.K_ESCAPE]: # exit game
                run = False
        
        # step environment
        obs, reward, done, info = env.step(action)

        # render environment
        env.render()
    
    # close gym environment
    env.render_close()
import pygame
import pymunk
import pymunk.pygame_util

import time
import random
import sys

pygame.init()

class Constants: # Change stuff about the simulation here

    # display/pygame stuff
    size = (1200, 800)
    per_frame_draw = 1 # redraw screen every {per_frame_draw} frames

    # body related stuff
    body_radius = 3
    body_mass = 1e6
    body_elasticity = 0.8
    body_friction = 0.6

    # sim related stuff
    bodies_N = 500
    collision_iterations = 30 # Increasing this doesn't significantly help with errors
    gravity_min_dist_sqrd = (2*body_radius)**2 # min distance squared where gravity will be applied
    gravity_max_dist_sqrd = 400**2 # max distance squared where gravity will be applied
    per_frame_gravity = 3 # recalculate gravity every {per_frame_gravity} frames
    dt = 0.1
    G = 6e-5

    # using a spatial hash didn't really improve performance, at least from what I tried
    use_spatial_hash = False
    spatial_hash_dim = 4*body_radius
    spatial_hash_count = bodies_N*10

    # FUCKING ANGULAR VELOCITY
    # Main problem of the simulation, colliding bodies seem to increase their angular velocity continuously
    # Haven't found a fix yet, as I believe its due to numerical errors ( increasing collision iterations doesn't help
    # as the problem persists unchanged even if they're cranked to 100-200 ), only thing to be done are these
    dampen_angular_velocity = False # dampen angular velocity every collision, so it eventually goes to 0
    angular_dampening = 0.9
    cap_angular_velocity = True # cap the angular velocity a body can have
    angular_cap = 0.1


class CustomBody(pymunk.Body): # for now its useless, later I may use it for adding temperature
    def __init__(self, mass):
        moment = pymunk.moment_for_circle(mass, 0, Constants.body_radius)
        super().__init__(mass, moment)

def custom_post_solve(arbiter, space, data): # JUST FOR HANDLING ANGULAR VELOCITY
    for shape in arbiter.shapes:
        if Constants.dampen_angular_velocity:
            shape.body.angular_velocity *= Constants.angular_dampening
        if Constants.cap_angular_velocity:
            if shape.body.angular_velocity >= 0:
                shape.body.angular_velocity = min(shape.body.angular_velocity, Constants.angular_cap)
            else:
                shape.body.angular_velocity = max(shape.body.angular_velocity, Constants.angular_cap)


def funky_gravity(bodies):
    # This monstrocity is my attempt at using list comprehensions for gravity force calculations, by abusing the := operator
    # which as it turns out is REALLY worse than the other method ( didn't continue it for this reason )

    forces = [diff_vec.normalized()*Constants.G*b1.mass*b2.mass/dist_sqrd for b2 in bodies for b1 in bodies if
              (dist_sqrd := (diff_vec := b2.position - b1.position).dot(diff_vec)) > Constants.gravity_min_dist_sqrd and
              dist_sqrd < Constants.gravity_max_dist_sqrd]

def gravity(bodies):
    # This gives the most performance out of all the things I tried
    # using numpy doesn't help at all from what I tested, it even makes the performances worse by a lot

    forces = [pymunk.Vec2d.zero()]*Constants.bodies_N

    for i, b1 in enumerate(bodies):
        for j, b2 in enumerate(bodies[i + 1:]):
            diff_vec = b2.position - b1.position
            dist_sqrd = diff_vec.dot(diff_vec) # faster than b1.position.get_dist_sqrd(b2.position)

            if (dist_sqrd < Constants.gravity_min_dist_sqrd or
                dist_sqrd > Constants.gravity_max_dist_sqrd):
                continue

            F_dir = diff_vec.normalized()
            F_mag = Constants.G*b1.mass*b2.mass/dist_sqrd
            F = F_dir*F_mag

            forces[i] += F
            forces[i + 1 + j] -= F

        # this instead of at local point helps a bit with errors
        b1.apply_force_at_world_point(forces[i], b1.position)

class Simulator:
    def __init__(self):

        self._display = pygame.display.set_mode(Constants.size)
        self._options = pymunk.pygame_util.DrawOptions(self._display)

        self.space = pymunk.Space()
        self.bodies = []

        self.gravity_func = gravity

        self.init_sim()

    def init_sim(self):

        # setup space
        self.space.gravity = (0, 0)
        self.space.iterations = Constants.collision_iterations
        handler = self.space.add_collision_handler(1, 1)
        handler.post_solve = custom_post_solve
        if Constants.use_spatial_hash:
            self.space.use_spatial_hash(
                dim=Constants.spatial_hash_dim,
                count=Constants.spatial_hash_count
            )

        # make all the bodies
        for _ in range(Constants.bodies_N):
            body = CustomBody(Constants.body_mass)
            body.position = (random.randint(0, Constants.size[0]),
                             random.randint(0, Constants.size[1]))

            shape = pymunk.Circle(body, radius=Constants.body_radius)
            shape.elasticity = Constants.body_elasticity
            shape.friction = Constants.body_friction
            shape.collision_type = 1

            self.bodies.append(body)
            self.space.add(body, shape)


    def start(self):

        total_frames = 0
        fps = 0

        t_start = time.time()
        t0 = t_start

        running = True
        while running:
            fps += 1
            total_frames += 1

            if time.time() - t0 >= 1:
                print('fps:', fps)
                fps = 0
                t0 = time.time()


            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)):
                    running = False

            if total_frames % Constants.per_frame_gravity == 0:
                self.gravity_func(self.bodies)

            if total_frames % Constants.per_frame_draw == 0:
                self._display.fill((0, 0, 0))
                self.space.debug_draw(self._options)
                pygame.display.flip()

            self.space.step(Constants.dt)

        return 0

if __name__ == "__main__":
    sim = Simulator()
    sys.exit(sim.start())





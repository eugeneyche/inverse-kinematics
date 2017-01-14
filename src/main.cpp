#include "draw.hpp"
#include "ik.hpp"
#include <cstdio>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

int window_width = 800;
int window_height = 600;
const char* window_title = "Inverse Kinematics";

struct Camera
{
    glm::mat4 projection;
    glm::mat4 view;
} cam;

float dolly_speed = 0.01f;
float dist_speed = 0.5f;

glm::dvec2 cursor;
float dist = 5.f;
float ax, ay;

glm::vec3 target;

void update_cam_view()
{
    glm::vec3 offset = {
        dist * glm::cos(ay) * glm::sin(ax),
        dist * glm::sin(ay),
        dist * glm::cos(ay) * glm::cos(ax),
    };
    cam.view = glm::lookAt(
        offset,
        glm::vec3{0.f, 0.f, 0.f},
        glm::vec3{0.f, 1.f, 0.f}
        );
}

void on_cursor_pos(GLFWwindow* window, double x, double y)
{
    float dx =  dolly_speed * -(x - cursor.x);
    float dy =  dolly_speed * (y - cursor.y);
    cursor = {x, y};

    float ay_extreme = glm::pi<float>() / 2.f - 0.1f;
    ax += dx;
    ay = glm::clamp(ay + dy, -ay_extreme, ay_extreme);
    update_cam_view();
}

void on_scroll(GLFWwindow* window, double x, double y)
{
    dist = glm::clamp(dist - static_cast<float>(y), 2.f, 50.f);
    update_cam_view();
}

void on_mouse_button(GLFWwindow* window, int button, int action, int mods)
{
    if (action == GLFW_PRESS) {
        glfwGetCursorPos(window, &cursor.x, &cursor.y);
        if (button == 0) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            glfwSetCursorPosCallback(window, on_cursor_pos);
        } else if (button == 1) {
            glm::mat4 inverse_pv = glm::inverse(cam.view) * glm::inverse(cam.projection);
            glm::vec3 pos = glm::vec3(inverse_pv * glm::vec4(
                2 * static_cast<float>(cursor.x) / window_width - 1,
                2 * static_cast<float>(cursor.y) / window_height - 1,
                0, 1
                ));
            glm::vec3 forward_pos = glm::vec3(inverse_pv * glm::vec4(
                2 * static_cast<float>(cursor.x) / window_width - 1,
                2 * static_cast<float>(cursor.y) / window_height - 1,
                1, 1
                ));
        }
    } else if (action == GLFW_RELEASE) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        glfwSetCursorPosCallback(window, nullptr);
    }
}

void on_key(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_W) {
            target += glm::vec3{0.f, 0.f, -1.f};
        }
        if (key == GLFW_KEY_A) {
            target += glm::vec3{-1.f, 0.f, 0.f};
        }
        if (key == GLFW_KEY_S) {
            target += glm::vec3{0.f, 0.f, 1.f};
        }
        if (key == GLFW_KEY_D) {
            target += glm::vec3{1.f, 0.f, 0.f};
        }
    }
}

void make_grid(std::vector<VertPC>& grid, int size, const glm::vec3& color)
{
    float half_size = size / 2.f;
    for (int i = 0; i <= size; i++) {
        grid.push_back({glm::vec3{-half_size + i, 0, -half_size}, color});
        grid.push_back({glm::vec3{-half_size + i, 0,  half_size}, color});
        grid.push_back({glm::vec3{-half_size, 0, -half_size + i}, color});
        grid.push_back({glm::vec3{ half_size, 0, -half_size + i}, color});
    }
}

void draw_joints(DrawUtil* du, const std::vector<Joint>& joints)
{
    glm::vec3 axis_color = {0.f, 0.f, 1.f};
    glm::vec3 offset_color = {1.f, 0.f, 0.f};
    glm::vec3 position;
    glm::mat3 basis {1.f};
    std::vector<VertPC> vertices;
    for (const auto& joint : joints) {
        vertices.push_back({position, axis_color});
        glm::vec3 axis = glm::normalize(basis * joint.axis);
        vertices.push_back({position + axis * 0.5f, axis_color});
        basis = basis * glm::mat3_cast(glm::rotate(glm::quat{}, joint.angle, joint.axis));

        vertices.push_back({position, offset_color});
        position += basis * joint.offset;
        vertices.push_back({position, offset_color});
    }
    du->draw(GL_LINES, cam.projection, cam.view, vertices);
    glPointSize(3.f);
    du->draw(GL_POINTS, cam.projection, cam.view, vertices);
}

void print_quat(const glm::quat& quat)
{
    printf("{%f, %f, %f, %f}\n", quat.w, quat.x, quat.y, quat.z);
}

int main()
{
    if (not glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW.\n");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    GLFWwindow* window = glfwCreateWindow(window_width, window_height, window_title, nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr, "Failed to create window.\n");
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (not gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        fprintf(stderr, "Failed to initialize GLAD.\n");
        return -1;
    }

    glfwSetMouseButtonCallback(window, on_mouse_button);
    glfwSetKeyCallback(window, on_key);
    glfwSetScrollCallback(window, on_scroll);

    ShaderManager sm;
    DrawUtil du {&sm};

    if (not du.init()) {
        fprintf(stderr, "Failed to initialize draw util.\n");
        return -1;
    }

    std::vector<VertPC> grid;
    make_grid(grid, 10, glm::vec3{0.3f, 0.3f, 0.3f});

    float aspect = static_cast<float>(window_width) / window_height;
    cam.projection = glm::perspective(1.f, aspect, 0.1f, 1000.f);
    update_cam_view();

    std::vector<Joint> joints = {
        Joint{{0.f, 1.f, 0.f}, {0.f, 1.f, 0.f}, 0.f},
        Joint{{0.f, 0.5f, 0.f}, {1.f, 0.f, 0.f}, 0.f},
        Joint{{0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}, 0.f},
        Joint{{0.f, 2.f, 0.f}, {1.f, 0.f, 0.f}, 0.f},
        Joint{{0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}, 0.f},
        Joint{{0.f, 0.5f, 0.f}, {1.f, 0.f, 0.f}, 0.f},
    };

    target = {2.f, 0.f, 2.f};

    glClearColor(0.5f, 0.5f, 0.5f, 0.f);
    glPointSize(5.f);
    while (not glfwWindowShouldClose(window)) {
        joints = solve_ik(joints, target, 0.01f, 100);
        float time = glfwGetTime();
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        du.draw(GL_LINES, cam.projection, cam.view, grid);
        glPointSize(5.f);
        du.draw(GL_POINTS, cam.projection, cam.view, {{target, {1.f, 1.f, 0.f}}});
        draw_joints(&du, joints);
        glfwSwapBuffers(window);
    }

    glfwTerminate();
}

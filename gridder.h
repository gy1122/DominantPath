#ifndef __DominantPath__gridder__
#define __DominantPath__gridder__


class Gridder {
public:
    class Segment;  // forward def so Iter can have reference
    class Iter {
    public:
        Iter(const Segment& segment, int x, int y) : segment_(segment),
                                                     cell_(x, y) {}

        bool operator==(const Iter& a) const {
            return cell_.first == a.cell_.first &&
                cell_.second == a.cell_.second;
        }
        bool operator!=(const Iter& a) const {
            return cell_.first != a.cell_.first ||
                cell_.second != a.cell_.second;
        }
        const std::pair<int,int>& operator*() const {
            return cell_;
        }
        Iter& operator++() {
            if (segment_.transpose_) {
                if (++cell_.first > segment_.y_to_x_hi(cell_.second)) {
                    cell_.first = segment_.y_to_x_lo(++cell_.second);
                }
            } else {
                if (++cell_.second > segment_.x_to_y_hi(cell_.first)) {
                    cell_.second = segment_.x_to_y_lo(++cell_.first);
                }
            }
            return *this;
        }
        Iter operator++(int) {
            Iter x(*this);
            ++*this;
            return x;
        }
    private:
        const Segment& segment_;
        std::pair<int,int> cell_;
    };

    class Segment {
    public:
        Iter begin() const {
            if (transpose_) {
                int y_lo = int(std::floor(std::min(y0_, y1_) - epsilon_));
                return Iter(*this, y_to_x_lo(y_lo), y_lo);
            } else {
                int x_lo = int(std::floor(std::min(x0_, x1_) - epsilon_));
                return Iter(*this, x_lo, x_to_y_lo(x_lo));
            }
        }
        Iter end() const {
            if (transpose_) {
                int y_hi = int(std::floor(std::max(y0_, y1_) + epsilon_)) + 1;
                return Iter(*this, y_to_x_lo(y_hi), y_hi);
            } else {
                int x_hi = int(std::floor(std::max(x0_, x1_) + epsilon_)) + 1;
                return Iter(*this, x_hi, x_to_y_lo(x_hi));
            }
        }
        double x_to_y(double x) const {
            if (transpose_) {
                return y0_ + (x - x0_) / slope_;
            } else {
                return y0_ + (x - x0_) * slope_;
            }
        }
        double y_to_x(double y) const {
            if (transpose_) {
                return x0_ + (y - y0_) * slope_;
            } else {
                return x0_ + (y - y0_) / slope_;
            }
        }
        int x_to_y_lo(int x) const {
            double x_lo = double(x) - epsilon_;
            double x_hi = double(x) + 1.0 + epsilon_;
            return int(std::floor(std::min(x_to_y(x_lo), x_to_y(x_hi))
                                  - epsilon_));
        }
        int x_to_y_hi(int x) const {
            double x_lo = double(x) - epsilon_;
            double x_hi = double(x) + 1.0 + epsilon_;
            return int(std::floor(std::max(x_to_y(x_lo), x_to_y(x_hi))
                                  + epsilon_));
        }
        int y_to_x_lo(int y) const {
            double y_lo = double(y) - epsilon_;
            double y_hi = double(y) + 1.0 + epsilon_;
            return int(std::floor(std::min(y_to_x(y_lo), y_to_x(y_hi))
                                  - epsilon_));
        }
        int y_to_x_hi(int y) const {
            double y_lo = double(y) - epsilon_;
            double y_hi = double(y) + 1.0 + epsilon_;
            return int(std::floor(std::max(y_to_x(y_lo), y_to_x(y_hi))
                                  + epsilon_));
        }

    private:
        friend class Gridder;
        friend class Iter;
        Segment(double x0, double y0, double x1, double y1, double epsilon)
            : x0_(x0), y0_(y0), x1_(x1), y1_(y1), epsilon_(epsilon),
              transpose_((std::abs(y1-y0) > std::abs(x1-x0))) {
            if (transpose_) {
                slope_ = (x1_ - x0_) / (y1_ - y0_);
            } else {
                slope_ = (y1_ - y0_) / (x1_ - x0_);
            }
        }

        double x0_;
        double y0_;
        double x1_;
        double y1_;
        double epsilon_;
        bool transpose_;
        double slope_;
    };

    explicit Gridder(double scale, double epsilon=1e-6) : scale_(scale),
                                                          epsilon_(epsilon) {}
    Segment segment(double x0, double y0, double x1, double y1) const {
        return Segment(x0*scale_, y0*scale_, x1*scale_, y1*scale_, epsilon_);
    }
private:
    double scale_;
    double epsilon_;
};

#endif  // __DominantPath__gridder__

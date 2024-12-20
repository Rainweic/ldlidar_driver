

/// 雷达点云数据结构
#[derive(Debug, Clone, Format)]
pub struct PointData {
    /// 角度
    pub angle: f32,
    /// 距离
    pub distance: u16,
    /// 强度
    pub intensity: u8,
    // 时间戳
    pub timestamp: u64,
}

/// 近距离滤波器配置
pub struct SlbfConfig {
    /// 高置信度阈值
    pub confidence_high: u16,
    /// 中等置信度阈值
    pub confidence_middle: u16,
    /// 低置信度阈值
    pub confidence_low: u16,
    /// 默认扫描频率(Hz)
    pub scan_freq: u16,
}

impl Default for SlbfConfig {
    fn default() -> Self {
        Self {
            confidence_high: 200,
            confidence_middle: 150,
            confidence_low: 92,
            scan_freq: 2300,
        }
    }
}

/// 近距离滤波器
/// 用于过滤掉近距离1m内的不合理点云数据
pub struct Slbf {
    /// 当前转速
    curr_speed: f32,
    /// 是否启用严格过滤策略
    enable_strict_policy: bool,
    /// 配置参数
    config: SlbfConfig,
}

impl Slbf {
    /// 创建新的滤波器实例
    pub fn new(speed: f32, strict_policy: bool) -> Self {
        Self {
            curr_speed: speed,
            enable_strict_policy: strict_policy,
            config: SlbfConfig::default(),
        }
    }

    /// 1米范围内的近距离滤波,过滤不合理的数据点
    /// 
    /// # Arguments
    /// * `points` - 输入的点云数据
    /// 
    /// # Returns
    /// * 过滤后的点云数据
    pub fn near_filter(&self, points: &[PointData]) -> Vec<PointData, 360> {
        // 分类存储点云数据
        let mut normal = Vec::new();   // 正常点
        let mut pending = Vec::new();  // 待处理点
        let mut item = Vec::new();     // 临时组
        let mut group = Vec::new();    // 有效组

        // 第一步:根据距离和强度进行初步分类
        for point in points {
            if point.distance == 0 {
                continue;
            }

            // 距离大于1000mm的点直接认为有效
            if point.distance > 1000 {
                normal.push(point.clone()).ok();
                continue;
            }

            // 根据强度进行分类
            if point.intensity > self.config.confidence_high {
                // 高置信度,直接认为有效
                normal.push(point.clone()).ok();
            } else if point.intensity > self.config.confidence_middle {
                // 中等置信度,需要进一步判断
                if self.enable_strict_policy {
                    pending.push(point.clone()).ok();
                } else {
                    normal.push(point.clone()).ok();
                }
            } else if point.intensity > self.config.confidence_low {
                // 低置信度,需要严格判断
                pending.push(point.clone()).ok();
            }
            // 置信度太低的点直接丢弃
        }

        // 第二步:对待定点进行分组处理
        if !pending.is_empty() {
            // 按角度排序
            pending.sort_by(|a, b| a.angle.partial_cmp(&b.angle).unwrap());

            // 分组处理
            for point in pending {
                if item.is_empty() {
                    item.push(point).ok();
                    continue;
                }

                // 计算角度差
                let mut angle_diff = (point.angle - item.last().unwrap().angle).abs();
                if angle_diff > (360.0 - point.angle + item.last().unwrap().angle) {
                    angle_diff = 360.0 - point.angle + item.last().unwrap().angle;
                }

                // 角度差小于阈值,加入同一组
                if angle_diff <= (self.curr_speed / self.config.scan_freq as f32 * 2.0) {
                    item.push(point).ok();
                } else {
                    // 角度差大于阈值,开始新的一组
                    if item.len() >= 3 {
                        // 当前组至少3个点才认为有效
                        for p in item.iter() {
                            group.push(p.clone()).ok();
                        }
                    }
                    item.clear();
                    item.push(point).ok();
                }
            }

            // 处理最后一组
            if item.len() >= 3 {
                for p in item.iter() {
                    group.push(p.clone()).ok();
                }
            }
        }

        // 第三步:将有效点合并
        for point in group {
            normal.push(point).ok();
        }

        // 按角度排序
        normal.sort_by(|a, b| a.angle.partial_cmp(&b.angle).unwrap());

        normal
    }

    /// 设置是否启用严格过滤策略
    pub fn set_strict_policy(&mut self, enable: bool) {
        self.enable_strict_policy = enable;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_basic() {
        let filter = Slbf::new(10.0, true);
        
        // 创建测试数据
        let mut points = Vec::<PointData, 16>::new();
        points.push(PointData {
            angle: 0.0,
            distance: 500,
            intensity: 220, // 高置信度
            stamp: 0,
        }).unwrap();
        
        let filtered = filter.near_filter(&points);
        assert_eq!(filtered.len(), 1);
    }

    #[test]
    fn test_filter_low_confidence() {
        let filter = Slbf::new(10.0, true);
        
        // 创建低置信度测试数据
        let mut points = Vec::<PointData, 16>::new();
        points.push(PointData {
            angle: 0.0,
            distance: 500,
            intensity: 80, // 低置信度
            stamp: 0,
        }).unwrap();
        
        let filtered = filter.near_filter(&points);
        assert_eq!(filtered.len(), 0); // 应该被过滤掉
    }
}
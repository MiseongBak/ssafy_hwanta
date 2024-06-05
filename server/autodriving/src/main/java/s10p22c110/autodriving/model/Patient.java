package s10p22c110.autodriving.model;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.Table;
import lombok.Getter;
import lombok.Setter;

@Entity
@Getter
@Setter
@Table(name = "patients")
public class Patient {
    
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;
    
    @Column(name = "lat")
    private String lat;

    @Column(name = "lon")
    private String lon;

    // 기본 생성자
    public Patient() {
    }

    // 모든 필드를 포함한 생성자
    public Patient(String lat, String lon) {
        this.lat = lat;
        this.lon = lon;
    }
}
